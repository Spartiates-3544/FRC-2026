package frc.mentor.logic;

import edu.wpi.first.math.geometry.Translation3d;
import frc.mentor.robot.Records;

/**
 * <p>
 * API simple pour viser le goal et shooter en mouvement.
 * </p>
 *
 * <p>
 * <b>Idée:</b>
 * </p>
 * <ul>
 * <li><b>BallisticSolver</b> = math</li>
 * <li><b>ShooterLogic</b> = helpers (géométrie + sim + blind spot)</li>
 * <li><b>ShooterAim</b> = API simple pour les jeunes</li>
 * <li><b>ShooterTuning</b> = tuning des paramètres en temps réel du shooter</li>
 * </ul>
 *
 * <p>
 * <b>Deux façons d'utiliser:</b>
 * </p>
 *
 * <p>
 * <b>(A) Mode simple</b> (recommandé pour les jeunes):
 * </p>
 * 
 * <pre>{@code
 * ShooterAim.Loop aim = new ShooterAim.Loop(Constants.SHOOTER_DEFAULTS);
 * ...
 * ShooterAim.AimResult out = aim.update(robotNow, actNow, targetXYZ, holdYawRelRad);
 * // envoyer out.cmd() aux moteurs
 * // feeder/indexer si out.ok()
 * }</pre>
 *
 * <p>
 * <b>(B) Mode avancé</b> / stateless:
 * </p>
 * 
 * <pre>{@code
 * ShooterAim.AimResult out = ShooterAim.solve(p, robotNow, actNow, targetXYZ, warm, holdYawRelRad);
 * warm = out.nextWarm();
 * }</pre>
 */
public final class ShooterAim {
    private ShooterAim() {
    }

    /** Résultat simple (facile à logger + à commander). */
    public static record AimResult(
            Records.ShotSolution desired,
            Records.ShotSolution cmd,
            boolean ok,
            boolean blindHold,
            Records.ShotSolution nextWarm,
            Records.Debug debug) {

        /** @return yaw désiré en degrés. */
        public double yawDesiredDeg() {
            return Math.toDegrees(desired.turretYawRelRad());
        }

        /** @return yaw commandé en degrés (avec blind hold). */
        public double yawCmdDeg() {
            return Math.toDegrees(cmd.turretYawRelRad());
        }

        /** @return rpm commandé. */
        public double rpmCmd() {
            return cmd.flywheelRpm();
        }

        /** @return hood commandé (deg). */
        public double hoodCmdDeg() {
            return cmd.hoodDeg();
        }

        /** @return miss estimé (m). */
        public double missM() {
            return cmd.missM();
        }
    }

    /**
     * <p>
     * Mode "simple" : garde le warm-start et lit les params live.
     * </p>
     *
     * <p>
     * Pattern:
     * - Construis 1 fois dans ton subsystem
     * - Appelle {@link #update(Records.RobotState, Records.ActuatorState, Translation3d, double)} chaque loop
     * </p>
     */
    public static final class Loop {
        private final Records.ShooterParams defaults;
        private Records.ShotSolution warm = null;
        private AimResult last = null;

        public Loop(Records.ShooterParams defaults) {
            this.defaults = defaults;
        }

        /** Dernier résultat (peut être null si update n'a jamais été appelé). */
        public AimResult last() {
            return last;
        }

        /** Reset warm-start (ex: disable/enable, changement de target, etc.). */
        public void resetWarm() {
            warm = null;
        }

        /**
         * Update one-call:
         * <ol>
         * <li>Lit params live via {@link ShooterTuning#params(Records.ShooterParams)}</li>
         * <li>Run le solve</li>
         * <li>Met à jour le warm automatiquement</li>
         * </ol>
         *
         * @param robotNow
         *            état robot actuel
         * @param actNow
         *            état actuateurs actuel
         * @param targetXYZ
         *            cible (field, m)
         * @param holdYawRelRad
         *            yaw à tenir si blind-spot (typiquement yaw actuel turret)
         * @return résultat à commander
         */
        public AimResult update(
                Records.RobotState robotNow,
                Records.ActuatorState actNow,
                Translation3d targetXYZ,
                double holdYawRelRad) {

            Records.ShooterParams pLive = ShooterTuning.params(defaults);
            last = ShooterAim.solve(pLive, robotNow, actNow, targetXYZ, warm, holdYawRelRad);
            warm = last.nextWarm();
            return last;
        }
    }

    /** Solve stateless (avancé / tests). */
    public static AimResult solve(
            Records.ShooterParams p,
            Records.RobotState robotNow,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm,
            double holdYawRelRad) {

        // 1) solve heavy
        Records.SolveOutput out = BallisticSolver.solve(p, robotNow, actNow, targetXYZ, warm);
        Records.ShotSolution desired = out.solution();

        // 2) blind spot check
        boolean blindHold = ShooterLogic.isTurretBlindSpot(p, desired.turretYawRelRad());

        // 3) yaw commandé (hold si interdit)
        double yawCmd = ShooterLogic.applyBlindHoldYaw(p, desired.turretYawRelRad(), holdYawRelRad);

        // 4) ok final (si blind hold: on gate feed/indexer)
        boolean okFinal = desired.ok() && !blindHold;

        // 5) cmd final
        Records.ShotSolution cmd = new Records.ShotSolution(
                okFinal,
                yawCmd,
                desired.flywheelRpm(),
                desired.hoodDeg(),
                desired.tFireS(),
                desired.tofS(),
                desired.missM(),
                blindHold ? "blind_hold" : desired.info());

        // 6) warm-start: on garde desired (pas cmd)
        Records.ShotSolution nextWarm = desired;

        return new AimResult(desired, cmd, okFinal, blindHold, nextWarm, out.debug());
    }
}
