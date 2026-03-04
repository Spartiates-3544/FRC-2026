package frc.lib.logic;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.logging.ExtendedLogger;
import frc.lib.robot.Records;

/**
 * ShooterAim = petite API pour "viser + shooter" pendant que le robot bouge.
 *
 * Idée générale :
 * - Tu appelles Loop.update() à chaque 20ms (50Hz).
 * - Ça prend l'état du robot + l'état des actionneurs + la position de la cible.
 * - Ça te sort un AimResult avec:
 * - desired = la solution "idéale" du solver (math pure)
 * - cmd = la commande finale (avec protections genre blind spot)
 * - ok = true si tu peux feed/shoot maintenant
 *
 * Comment l'utiliser (dans ton Subsystem/Command) :
 *
 * // 1) crée la loop 1 fois (ex: dans le Subsystem)
 * private final ShooterAim.Loop aimLoop = new ShooterAim.Loop();
 *
 * // 2) dans periodic() ou execute() à 50Hz (loop de 20ms):
 * var res = aimLoop.update(robotState, actuatorState, targetXYZ, holdYawRelRad);
 *
 * // 3) applique res.cmd() aux subsystems:
 * turret.setYawRelRad(res.cmd().turretYawRelRad());
 * shooter.setRpm(res.cmd().flywheelRpm());
 * hood.setDeg(res.cmd().hoodDeg());
 *
 * // 4) gate le feed/indexer:
 * if (res.ok()) { indexer.feed(); } else { indexer.stop(); }
 */
public final class ShooterAim {
    private ShooterAim() {
    }

    /**
     * Résultat prêt à consommer.
     *
     * - desired: sortie brute du solver (ce que la physique voudrait)
     * - cmd: sortie "safe" (blind spot / hold / etc)
     * - ok: autorisation de tirer/feeder (si false => tu bloques l'indexer et le kicker (peut etre aussi l'intake))
     * - blindHold: true si on est dans une zone interdite (turret blind spot)
     * - nextWarm: warm-start pour stabiliser la prochaine solve (évite les jumps)
     * - debug: infos pour logger / comprendre (facultatif)
     */
    public static record AimResult(
            Records.ShotSolution desired,
            Records.ShotSolution cmd,
            boolean ok,
            boolean blindHold,
            Records.ShotSolution nextWarm,
            Records.Debug debug) {

        /** yaw voulu (desired) en degrés pour debug/telemetry */
        public double yawDesiredDeg() {
            return Math.toDegrees(desired.turretYawRelRad());
        }

        /** yaw commandé (cmd) en degrés pour debug/telemetry */
        public double yawCmdDeg() {
            return Math.toDegrees(cmd.turretYawRelRad());
        }

        /** RPM commandé (cmd) */
        public double rpmCmd() {
            return cmd.flywheelRpm();
        }

        /** Hood commandé (cmd) */
        public double hoodCmdDeg() {
            return cmd.hoodDeg();
        }

        /** "miss" estimé (cmd) */
        public double missM() {
            return cmd.missM();
        }
    }

    /**
     * Mode simple (recommandé) :
     * - garde un warm-start interne
     * - lit les params live (ShooterTuning) mais PAS à 50Hz (throttled)
     *
     * Pourquoi throttled ?
     * - Le solver tourne à 50Hz.
     * - Refaire ShooterParams via NT/reflection à 50Hz = alloc + CPU inutile.
     * - Donc on refresh les params genre à 10Hz.
     */
    public static final class Loop {

        // Ces champs sont lus par ton logger en arrière-plan (DogLog / ExtendedLogger)
        @ExtendedLogger.LoggableField(path = "ShooterAim/liveParams", hz = 2)
        private volatile Records.ShooterParams liveParamsLogged = null;

        @ExtendedLogger.LoggableField(path = "ShooterAim/last", hz = 2)
        private volatile AimResult lastLogged = null;

        @ExtendedLogger.LoggableField(path = "ShooterAim/holdYawRelDeg", hz = 20)
        private volatile double holdYawRelDegLogged = 0.0;

        // Warm-start: la dernière solution "desired" qu'on réinjecte au solve suivant
        private Records.ShotSolution warm = null;

        // Dernier résultat complet
        private AimResult last = null;

        // Refresh params à 10Hz
        private static final double PARAMS_REFRESH_S = 0.10;
        private Records.ShooterParams cachedParams = null;
        private double lastParamsReadS = -1.0;

        public Loop() {
            ExtendedLogger.registerInstance(this);
        }

        /** pour récupérer le dernier résultat sans recalculer */
        public AimResult last() {
            return last;
        }

        /** si tu veux "reset" le warm-start (ex: gros changement d'état / target) */
        public void resetWarm() {
            warm = null;
        }

        /**
         * Lit ShooterParams (NT) mais seulement à une fréquence raisonnable.
         * Le reste du temps on réutilise cachedParams.
         */
        private Records.ShooterParams getParamsCached() {
            double now = Timer.getFPGATimestamp();

            if (cachedParams == null || lastParamsReadS < 0.0) {
                cachedParams = ShooterTuning.get().params();
                lastParamsReadS = now;
                return cachedParams;
            }

            if ((now - lastParamsReadS) >= PARAMS_REFRESH_S) {
                cachedParams = ShooterTuning.get().params();
                lastParamsReadS = now;
            }

            return cachedParams;
        }

        /**
         * Appel principal à 50Hz.
         *
         * Inputs importants :
         * - robotNow: pose/vitesse/etc du robot
         * - actNow: états actuels turret/shooter/hood (si ton solver en a besoin)
         * - targetXYZ: position cible en coord robot/field (selon ton BallisticSolver)
         * - holdYawRelRad: yaw à maintenir si on rentre en blind spot
         *
         * Output :
         * - AimResult res
         * - res.cmd() = ce que tu commandes
         * - res.ok() = autorise feed/shoot
         */
        public AimResult update(
                Records.RobotState robotNow,
                Records.ActuatorState actNow,
                Translation3d targetXYZ,
                double holdYawRelRad) {

            Records.ShooterParams pLive = getParamsCached();

            last = ShooterAim.solve(pLive, robotNow, actNow, targetXYZ, warm, holdYawRelRad);

            // Warm-start pour la prochaine frame
            warm = last.nextWarm();

            // snapshots pour logs
            liveParamsLogged = pLive;
            lastLogged = last;
            holdYawRelDegLogged = Math.toDegrees(holdYawRelRad);

            return last;
        }
    }

    /**
     * Solve stateless (mode avancé / tests unitaires).
     *
     * Tu l'utilises si :
     * - tu veux gérer toi-même warm-start
     * - tu veux passer un ShooterParams spécifique
     * - tu veux tester BallisticSolver sans Loop
     */
    public static AimResult solve(
            Records.ShooterParams p,
            Records.RobotState robotNow,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm,
            double holdYawRelRad) {

        // 1) solve "heavy" (balistique) -> donne une solution idéale
        Records.SolveOutput out = BallisticSolver.solve(p, robotNow, actNow, targetXYZ, warm);
        Records.ShotSolution desired = out.solution();

        // 2) check blind spot du turret
        boolean blindHold = ShooterLogic.isTurretBlindSpot(p, desired.turretYawRelRad());

        // 3) calc yaw commandé :
        // - si on est dans blind spot, on force un "hold" (yaw safe)
        double yawCmd = ShooterLogic.applyBlindHoldYaw(p, desired.turretYawRelRad(), holdYawRelRad);

        // 4) ok final :
        // - ok du solver (ballistics ok)
        // - ET pas en blind hold (sinon tu bloques feed)
        boolean okFinal = desired.ok() && !blindHold;

        // 5) construit la commande finale (cmd)
        Records.ShotSolution cmd = new Records.ShotSolution(
                okFinal,
                yawCmd,
                desired.flywheelRpm(),
                desired.hoodDeg(),
                desired.tFireS(),
                desired.tofS(),
                desired.missM(),
                blindHold ? "blind_hold" : desired.info());

        // 6) warm-start = on garde desired (pas cmd)
        Records.ShotSolution nextWarm = desired;

        return new AimResult(desired, cmd, okFinal, blindHold, nextWarm, out.debug());
    }
}