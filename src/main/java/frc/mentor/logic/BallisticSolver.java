// frc/mentor/logic/BallisticSolver.java
package frc.mentor.logic;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.DoubleUnaryOperator;

import frc.mentor.robot.Records;
import frc.mentor.utils.MathUtils;

/**
 * <p>
 * Solver balistique: shoot while moving.
 * </p>
 *
 * Le but:
 * - Prendre l'état du robot (pose + vitesse + accel + omega)
 * - Prendre l'état actuel des actuateurs (turret/hood/flywheel)
 * - Sortir une solution (yaw/rpm/hood) qui minimise le miss sur la cible
 *
 * Ce solver est "stateless":
 * - on peut passer une solution warm-start (solution de la loop précédente)
 * - sinon il s'arrange avec un seed + un guess de RPM/hood
 *
 * Notes importantes:
 * - Ici on sort des valeurs "desired" (solution idéale dans l'espace permis).
 * - La logique de blind spot turret n'est PAS ici: elle se fait au call-site
 * (ex: sur le robot ou dans SolverTest) en "blind-hold".
 */
public final class BallisticSolver {
    private BallisticSolver() {
    }

    // -----------------------------
    // Poids de continuité
    // -----------------------------
    // Ces poids disent au solver: "si deux solutions miss autant, préfère celle qui change moins vs la solution précédente".
    private static final double CONT_RPM_WEIGHT = 0.05;
    private static final double CONT_HOOD_WEIGHT = 2.50;
    private static final double CONT_YAW_WEIGHT = 1.00;

    // Si on est proche des limites RPM, on préfère utiliser le hood pour ajuster (plus stable)
    private static final double RPM_NEAR_LIMIT_BAND = 150.0;

    // L'amplitude de recherche du hood est scaled par cette valeur (sensibilité du hood)
    private static final double HOOD_SPAN_SCALE = 0.45;

    // Empêche le solver de demander des sauts de RPM trop petits (ça aide la stabilité)
    private static final double MIN_RPM_STEP_PER_TICK = 50.0;

    /**
     * <p>
     * Prédit l'état du robot après un temps t.
     * </p>
     *
     * Modèle:
     * - position: p + v*t + 0.5*a*t^2
     * - vitesse: v + a*t
     * - yaw: yaw + omega*t
     *
     * @param r
     *            état du robot maintenant
     * @param t
     *            temps dans le futur (s)
     * @return nouvel état prédit après t secondes
     */
    public static Records.RobotState predictRobot(Records.RobotState r, double t) {
        Translation2d dp = r.velXY().times(t).plus(r.accelXY().times(0.5 * t * t));
        Translation2d pos = r.posXY().plus(dp);

        Translation2d vel = r.velXY().plus(r.accelXY().times(t));
        Rotation2d yaw = r.yaw().plus(Rotation2d.fromRadians(r.omegaRadS() * t));

        return new Records.RobotState(pos, yaw, vel, r.omegaRadS(), r.accelXY());
    }

    /**
     * <p>
     * Estime le temps (s) avant d'être prêt à shoot.
     * </p>
     *
     * Regarde combien il reste à bouger:
     * - turret: |deltaYaw| / slewRate
     * - hood: |deltaDeg| / hoodRate
     * - rpm: |deltaRpm| / flywheelAccel
     *
     * On prend le max des 3, parce que faut que TOUT soit prêt pour shoot et que ça entre dans but.
     *
     * @param p
     *            paramètres du shooter
     * @param act
     *            état actuel des actuateurs
     * @param desired
     *            solution désirée (setpoints)
     * @return temps estimé (s) pour atteindre les setpoints (le plus lent des 3)
     */
    public static double estimateTimeToReady(
            Records.ShooterParams p,
            Records.ActuatorState act,
            Records.ShotSolution desired) {

        double dyaw = Math.abs(MathUtils.wrapRad(desired.turretYawRelRad() - act.turretYawRelRad()));
        double dhood = Math.abs(desired.hoodDeg() - act.hoodDeg());
        double drpm = Math.abs(desired.flywheelRpm() - act.flywheelRpm());

        double tYaw = dyaw / Math.max(1e-6, Math.toRadians(p.turretSlewRateDps()));
        double tHood = dhood / Math.max(1e-6, p.hoodRateDps());
        double tRpm = drpm / Math.max(1e-6, p.flywheelAccelRpmS());

        return Math.max(tYaw, Math.max(tHood, tRpm));
    }

    /**
     * <p>
     * Résout le problème complet "tirer en mouvement".
     * </p>
     *
     * Stratégie:
     * 1) On assume qu'on va tirer après la latence de tir (fireLatencyS)
     * 2) On prédit le robot à ce moment
     * 3) On solve instantanément yaw/rpm/hood pour l'état du robot actuel
     * 4) On estime combien de temps il faut pour être ready (turret/hood/flywheel)
     * 5) On met à jour tFire = latency + tReady
     * 6) On refait une 2e passe (ça converge vite et c'est cheap)
     *
     * @param p
     *            paramètres du shooter/solver
     * @param robotNow
     *            état robot actuel (pose/vitesse/accel/omega)
     * @param actNow
     *            état actuel des actuateurs (turret/hood/rpm)
     * @param targetXYZ
     *            position de la cible dans le field (m)
     * @param warm
     *            solution précédente (peut être null). Sert à stabiliser + warm-start.
     * @return SolveOutput contenant la solution + debug détaillé
     */
    public static Records.SolveOutput solve(
            Records.ShooterParams p,
            Records.RobotState robotNow,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm) {

        double tFire = p.fireLatencyS();
        Records.ShotSolution sol = null;
        Records.Debug dbg = null;

        // 2 passes
        for (int i = 0; i < 2; i++) {
            // Prédit le moment où on pense tirer
            Records.RobotState pred = predictRobot(robotNow, tFire);

            // Solve avec :
            // - Une recherche locale (https://elib.dlr.de/73550/1/Lampariello.pdf en partie, plein de choses que j'ai pas check)
            // - Section dorée (https://homepages.math.uic.edu/~jan/mcs471/goldensection.pdf)
            SolvePoseOut poseOut = solveForPoseInstant(p, pred, actNow, targetXYZ, (sol == null) ? warm : sol);
            PoseSolution ps = poseOut.sol();

            // Solution candidate
            sol = new Records.ShotSolution(
                    ps.ok(),
                    ps.turretYawRelRad(),
                    ps.flywheelRpm(),
                    ps.hoodDeg(),
                    tFire,
                    ps.tofS(),
                    ps.missM(),
                    ps.info());

            // Ajuste tFire avec le temps que ça prend pour atteindre les setpoints
            double tReady = estimateTimeToReady(p, actNow, sol);
            tFire = p.fireLatencyS() + tReady;

            // Debug: état robot au moment final du tir
            Records.RobotState predFinal = predictRobot(robotNow, tFire);

            dbg = new Records.Debug(
                    tReady,
                    tFire,
                    predFinal,

                    poseOut.seedYawRad(),
                    poseOut.distXY(),

                    poseOut.hit(),
                    poseOut.missM(),
                    ShooterLogic.rpmToExitSpeed(p, sol.flywheelRpm()),

                    poseOut.turretClamped(),
                    poseOut.rpmClamped(),
                    poseOut.hoodClamped(),

                    poseOut.yawCmdDeg(),
                    poseOut.yawRawDeg(),

                    poseOut.rpmCmd(),
                    poseOut.rpmRaw(),

                    poseOut.hoodCmdDeg(),
                    poseOut.hoodRawDeg(),

                    poseOut.missXY(),
                    poseOut.missZ(),

                    poseOut.sideEntry(),
                    p.goalType(),
                    sol.info());
        }

        return new Records.SolveOutput(sol, dbg);
    }

    /**
     * <p>
     * Solution interne pour une pose robot donnée.
     * </p>
     *
     * Ça contient plus d'infos que ShotSolution:
     * - hit/side entry
     * - missXY/missZ
     * - infos debug
     * 
     * Utile pour debug ou pour extend l'algo
     */
    private static record PoseSolution(
            boolean ok,
            double turretYawRelRad,
            double flywheelRpm,
            double hoodDeg,
            double tofS,
            double missM,
            double missXY,
            double missZ,
            boolean hit,
            boolean sideEntry,
            String info) {
    }

    /**
     * <p>
     * Output interne du solve instantané.
     * </p>
     *
     * En plus de la solution:
     * - seedYaw et distXY pour tuning/debug
     * - flags "clamped" pour comprendre pourquoi une solution est limitée
     * - raw vs cmd pour yaw/rpm/hood
     */
    private static record SolvePoseOut(
            PoseSolution sol,
            double seedYawRad,
            double distXY,

            boolean turretClamped,
            boolean rpmClamped,
            boolean hoodClamped,

            double yawRawDeg,
            double yawCmdDeg,

            double rpmRaw,
            double rpmCmd,

            double hoodRawDeg,
            double hoodCmdDeg) {

        /** @return miss total (m). */
        double missM() {
            return sol.missM();
        }

        /** @return composante XY du miss (m). */
        double missXY() {
            return sol.missXY();
        }

        /** @return composante Z du miss (m). */
        double missZ() {
            return sol.missZ();
        }

        /** @return true si la sim indique un hit. */
        boolean hit() {
            return sol.hit();
        }

        /** @return true si on a détecté une entrée par le côté (top entry). */
        boolean sideEntry() {
            return sol.sideEntry();
        }
    }

    /**
     * <p>
     * Résout yaw/rpm/hood pour une pose robot fixée.
     * </p>
     *
     * Plan:
     * - Guess initial: warm-start si dispo, sinon des valeurs raisonnables calculées
     * - Seed yaw: basé sur géométrie + vitesse robot + vitesse horizontale de sortie
     * - Définir une fenêtre de recherche (yawWin) et des spans (rpmSpan/hoodSpan)
     * - Faire des passes de raffinement:
     * - refineYaw (golden section sur miss)
     * - refineRpm (golden section sur miss)
     * - refineHood (golden section sur miss) selon conditions
     * - Clamp final (limites mécaniques + slew rpm par tick)
     * - Simuler la trajectoire finale -> miss/hit
     *
     * @param p
     *            paramètres shooter/solver
     * @param robot
     *            état du robot au moment du tir (déjà prédit)
     * @param actNow
     *            état actuel actuateurs (sert surtout pour limiter rpm par tick)
     * @param targetXYZ
     *            cible sur le field
     * @param warm
     *            solution précédente pour stabiliser (peut être null)
     * @return SolvePoseOut (solution + plein de debug)
     */
    private static SolvePoseOut solveForPoseInstant(
            Records.ShooterParams p,
            Records.RobotState robot,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm) {

        // -----------------------------
        // 1) Initialisation : on guess le hood/rpm initial idéal
        // -----------------------------
        double hoodGuessVar = (warm != null) ? warm.hoodDeg() : p.hoodFixedDeg();
        double rpmGuessVar = (warm != null) ? warm.flywheelRpm()
                : 0.5 * (p.flywheelRpmMin() + p.flywheelRpmMax());

        hoodGuessVar = MathUtils.clamp(hoodGuessVar, p.hoodMinDeg(), p.hoodMaxDeg());
        rpmGuessVar = MathUtils.clamp(rpmGuessVar, p.flywheelRpmMin(), p.flywheelRpmMax());

        // Si on ne solve pas le hood, on force un hood fixe
        if (!p.solveForHood()) {
            hoodGuessVar = MathUtils.clamp(p.hoodFixedDeg(), p.hoodMinDeg(), p.hoodMaxDeg());
        }

        // Si on solve le RPM, on peut faire un guess sans résistance de l'air pour aider à converger plus vite
        if (p.solveForRpm()) {
            Double g = rpmGuessNoDrag(p, robot, hoodGuessVar, targetXYZ);
            if (g != null)
                rpmGuessVar = 0.70 * g + 0.30 * rpmGuessVar;
        }

        final double hoodGuess = hoodGuessVar;
        final double rpmGuess = rpmGuessVar;

        // -----------------------------
        // 2) Seed du yaw (prend en compte v_robot)
        // -----------------------------
        double seedYaw = yawSeed(p, robot, targetXYZ, rpmGuess, hoodGuess);

        // Warm-start: on blend warm yaw et seed yaw (ça enlève du jitter selon mes tests)
        if (warm != null) {
            seedYaw = ShooterLogic.clampTurretYaw(
                    p, MathUtils.wrapRad(0.78 * warm.turretYawRelRad() + 0.22 * seedYaw));
        }
        final double seedYawFinal = seedYaw;

        // Selon la distance du muzzle on trouve le target à atteindre comparé au robot (XY)
        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);
        final double dist = Math.hypot(
                targetXYZ.getX() - muzzle.getX(),
                targetXYZ.getY() - muzzle.getY());

        // -----------------------------
        // 3) Fenêtres de recherche (adaptif)
        // -----------------------------
        // yawWin: plus loin = fenêtre plus petite
        final double yawWinDeg = MathUtils.clamp(
                6.5 * (4.0 / Math.max(dist, 1.0)),
                p.instantYawWinDegMin(),
                p.instantYawWinDegMax());
        final double yawWin = Math.toRadians(yawWinDeg);

        // rpmSpan: plus loin = span plus grand (besoin d'explorer plus)
        final double rpmSpan = MathUtils.clamp(
                p.instantRpmSpanMin() + 170.0 * dist,
                p.instantRpmSpanMin(),
                p.instantRpmSpanMax());

        // hoodSpan: plus loin = span plus petit (et scale global défini en haut (default 0.45))
        final double hoodSpan = HOOD_SPAN_SCALE * MathUtils.clamp(
                3.2 * (4.0 / Math.max(dist, 1.0)),
                p.instantHoodSpanMin(),
                p.instantHoodSpanMax());

        // Warm values (référence pour la continuité de la solution)
        final double warmYaw = (warm != null) ? warm.turretYawRelRad() : seedYawFinal;
        final double warmRpm = (warm != null) ? warm.flywheelRpm() : rpmGuess;
        final double warmHood = (warm != null) ? warm.hoodDeg() : hoodGuess;

        // -----------------------------
        // 4) Limites de spikes en RPM
        // -----------------------------
        // Empêche les gros sauts de RPM d'une loop à l'autre (peut rendre la commande weird et tire du jus au robot car 3 x60 drivent le shooter)
        final double rpmJumpWindowS = 0.10;
        final double maxRpmJumpPerSolve = Math.max(250.0, p.flywheelAccelRpmS() * rpmJumpWindowS);

        // Limite par tick de simulateur pour simuler un slew (ça évite de choisir un rpm qui serait fou)
        final double dtTick = Math.max(1e-3, p.rtDt());
        final double maxRpmStepPerTick = Math.max(MIN_RPM_STEP_PER_TICK, p.flywheelAccelRpmS() * dtTick);

        // Base RPM pour appliquer la limite de slew
        final double baseRpmForSlew = (warm != null) ? warm.flywheelRpm() : actNow.flywheelRpm();

        // -----------------------------
        // 5) Préférence du hood (tie-breaker)
        // -----------------------------
        final double hoodRange = Math.max(1e-6, (p.hoodMaxDeg() - p.hoodMinDeg()));
        final double hoodPreferSign = p.hoodPreferHigh() ? 1.0 : -1.0;

        // -----------------------------
        // 6) Structure : Best (meilleure solution trouvée)
        // -----------------------------
        class Best {
            PoseSolution sol = new PoseSolution(
                    false, seedYawFinal, rpmGuess, hoodGuess,
                    0.0, 1e9, 1e9, 1e9,
                    false, false,
                    "instant");
            double cost = 1e30;

            boolean turretClamped = false;
            boolean rpmClamped = false;
            boolean hoodClamped = false;

            double yawRawDeg = Math.toDegrees(seedYawFinal);
            double yawCmdDeg = Math.toDegrees(seedYawFinal);

            double rpmRaw = rpmGuess;
            double rpmCmd = rpmGuess;

            double hoodRawDeg = hoodGuess;
            double hoodCmdDeg = hoodGuess;
        }
        Best best = new Best();

        // -----------------------------
        // 7) Fonction de coût (ça se corse ici lol)
        // -----------------------------
        // cost = miss + continuityWeight * (différence du yaw/rpm/hood vs warm)
        java.util.function.Function<PoseSolution, Double> costFn = (s) -> {
            double dy = MathUtils.angleDiffRad(s.turretYawRelRad(), warmYaw);
            double dr = (s.flywheelRpm() - warmRpm) / 1000.0;
            double dh = (s.hoodDeg() - warmHood) / 10.0;

            double cont = (CONT_YAW_WEIGHT * dy * dy)
                    + (CONT_RPM_WEIGHT * dr * dr)
                    + (CONT_HOOD_WEIGHT * dh * dh);

            double c = s.missM() + p.continuityWeight() * cont;

            // Option: pousser le hood vers haut/bas si plusieurs solutions se valent (étant donné qu'on veux généralement un angle de hood plus haut)
            if (p.solveForHood() && p.hoodPreferWeight() > 0.0) {
                double hn = (s.hoodDeg() - p.hoodMinDeg()) / hoodRange;
                c -= hoodPreferSign * p.hoodPreferWeight() * hn;
            }
            return c;
        };

        // Compare deux coûts, avec tie-break du hood si deux cost sont très très proche
        java.util.function.BiPredicate<Double, Double> betterCost = (c, hoodVal) -> {
            if (c < best.cost - p.hoodTiebreakEps())
                return true;

            if (Math.abs(c - best.cost) <= p.hoodTiebreakEps() && p.solveForHood()) {
                if (p.hoodPreferHigh())
                    return hoodVal > best.sol.hoodDeg();
                return hoodVal < best.sol.hoodDeg();
            }
            return false;
        };

        // Nombre de passes de raffinement (clampé a une ou plus)
        final int passes = Math.max(1, p.instantIters());

        // Variables locales qu'on raffine
        double yawLocal = seedYawFinal;
        double rpmLocal = rpmGuess;
        double hoodLocal = hoodGuess;

        // Fenêtres locales (peuvent s'étendre plus tard)
        double yawWinLocal = yawWin;
        double rpmSpanLocal = rpmSpan;
        double hoodSpanLocal = hoodSpan;

        // Option: widen la recherche si on miss
        final int widenPasses = (p.instantWidenIfMiss() ? Math.max(0, p.instantWidenPasses()) : 0);

        // -----------------------------
        // 8) Loop principale du solveur
        // -----------------------------
        for (int widen = 0; widen <= widenPasses; widen++) {
            for (int it = 0; it < passes; it++) {

                // 8.1) Raffine yaw pour minimiser miss (rpm/hood fixés)
                yawLocal = refineYaw(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, yawWinLocal);

                // 8.2) Raffine RPM si activé
                if (p.solveForRpm()) {
                    // Petit helper: guess no-drag pour accélérer convergence
                    Double g = rpmGuessNoDrag(p, robot, hoodLocal, targetXYZ);
                    if (g != null)
                        rpmLocal = 0.80 * g + 0.20 * rpmLocal;

                    rpmLocal = refineRpm(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, rpmSpanLocal);

                    // Limite de jump vs warm
                    rpmLocal = MathUtils.clamp(rpmLocal, warmRpm - maxRpmJumpPerSolve, warmRpm + maxRpmJumpPerSolve);
                }

                // 8.3) Raffiner l'angle du hood si activé
                if (p.solveForHood()) {
                    // Si on ne solve pas RPM, le hood sert plus à corriger.
                    boolean rpmSolveOff = !p.solveForRpm();

                    // Si RPM proche des limites, le hood devient plus utile que pousser RPM plus loin
                    boolean rpmNearLimit = (rpmLocal >= p.flywheelRpmMax() - RPM_NEAR_LIMIT_BAND)
                            || (rpmLocal <= p.flywheelRpmMin() + RPM_NEAR_LIMIT_BAND);

                    if (rpmSolveOff || rpmNearLimit) {
                        hoodLocal = refineHood(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, hoodSpanLocal);
                        hoodLocal = MathUtils.clamp(hoodLocal, p.hoodMinDeg(), p.hoodMaxDeg());

                        // Après bouger le hood, on peut rerafiner RPM un peu (si actif)
                        if (p.solveForRpm()) {
                            Double g2 = rpmGuessNoDrag(p, robot, hoodLocal, targetXYZ);
                            if (g2 != null)
                                rpmLocal = 0.70 * g2 + 0.30 * rpmLocal;

                            rpmLocal = refineRpm(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal,
                                    0.55 * rpmSpanLocal);
                            rpmLocal = MathUtils.clamp(rpmLocal, warmRpm - maxRpmJumpPerSolve,
                                    warmRpm + maxRpmJumpPerSolve);
                        }
                    }
                }
            }

            // -----------------------------
            // 9) Clamp final + sim finale
            // -----------------------------
            double yawRaw = yawLocal;
            double rpmRaw = rpmLocal;
            double hoodRaw = hoodLocal;

            // Clamp mécaniques
            double yawCmd = ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(yawRaw));
            double hoodCmd = MathUtils.clamp(hoodRaw, p.hoodMinDeg(), p.hoodMaxDeg());

            // Clamp de RPM + slew par tick
            double rpmCmd = MathUtils.clamp(rpmRaw, p.flywheelRpmMin(), p.flywheelRpmMax());
            rpmCmd = MathUtils.clamp(rpmCmd, baseRpmForSlew - maxRpmStepPerTick, baseRpmForSlew + maxRpmStepPerTick);

            // Flags de debug
            boolean turretClamped = Math.abs(MathUtils.wrapRad(yawCmd - yawRaw)) > Math.toRadians(0.02);
            boolean hoodClamped = Math.abs(hoodCmd - hoodRaw) > 1e-6;
            boolean rpmClamped = Math.abs(rpmCmd - rpmRaw) > 1e-6;

            // Simulation finale pour mesurer si miss/hit
            Records.SimResult sim = ShooterLogic.simulateMissFast(p, robot, yawCmd, rpmCmd, hoodCmd, targetXYZ);

            PoseSolution cand = new PoseSolution(
                    sim.hit() || sim.missM() <= p.hitRadiusM(), // ok
                    yawCmd,
                    rpmCmd,
                    hoodCmd,
                    sim.t(),
                    sim.missM(),
                    sim.missXY_m(),
                    sim.missZ_m(),
                    sim.hit(),
                    sim.sideEntry(),
                    sim.hit() ? "hit" : "instant");

            double c = costFn.apply(cand);
            if (betterCost.test(c, cand.hoodDeg())) {
                best.cost = c;
                best.sol = cand;

                best.turretClamped = turretClamped;
                best.rpmClamped = rpmClamped;
                best.hoodClamped = hoodClamped;

                best.yawRawDeg = Math.toDegrees(yawRaw);
                best.yawCmdDeg = Math.toDegrees(yawCmd);

                best.rpmRaw = rpmRaw;
                best.rpmCmd = rpmCmd;

                best.hoodRawDeg = hoodRaw;
                best.hoodCmdDeg = hoodCmd;
            }

            // -----------------------------
            // 10) Widen si on miss
            // -----------------------------
            if (widen < widenPasses && best.sol.missM() > p.hitRadiusM()) {
                yawWinLocal = Math.min(
                        yawWinLocal * p.instantWidenYawMul(),
                        Math.toRadians(p.instantYawWinDegMax()));

                if (p.solveForRpm()) {
                    rpmSpanLocal = Math.min(rpmSpanLocal * p.instantWidenRpmMul(), p.instantRpmSpanMax());
                }
            }
        }

        // Output final
        return new SolvePoseOut(
                best.sol,
                seedYawFinal,
                dist,

                best.turretClamped,
                best.rpmClamped,
                best.hoodClamped,

                best.yawRawDeg,
                best.yawCmdDeg,

                best.rpmRaw,
                best.rpmCmd,

                best.hoodRawDeg,
                best.hoodCmdDeg);
    }

    /**
     * <p>
     * Raffine le yaw en minimisant le miss sur une fenêtre.
     * </p>
     *
     * Méthode:
     * - On encadre un minimum avec quelques samples
     * - Ensuite golden section pour converger sur une solution optimale
     *
     * @param p
     *            paramètres
     * @param robot
     *            état robot au moment du tir
     * @param target
     *            cible
     * @param yaw
     *            yaw actuel (rad)
     * @param rpm
     *            rpm actuel (RPM)
     * @param hood
     *            hood actuel (deg)
     * @param yawWin
     *            demi largeur de la fenêtre de recherche (rad)
     * @return yaw raffiné (rad) clampé aux limites turret
     */
    private static double refineYaw(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d target,
            double yaw,
            double rpm,
            double hood,
            double yawWin) {

        double loU = yaw - yawWin;
        double hiU = yaw + yawWin;

        // Fonction objectif: miss en fonction du yaw
        DoubleUnaryOperator f = (yyU) -> {
            double yy = ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(yyU));
            return ShooterLogic.simulateMissFast(p, robot, yy, rpm, hood, target).missM();
        };

        MathUtils.Bracket br = MathUtils.bracketFromSamples(loU, hiU, p.instantYawSamples(), f);
        MathUtils.MinResult mr = MathUtils.goldenSectionMinimize(f, br.a(), br.b(), p.instantYawGoldenIters());
        return ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(mr.x()));
    }

    /**
     * <p>
     * Raffine le RPM en minimisant le miss sur un span donné.
     * </p>
     *
     * @param p
     *            paramètres
     * @param robot
     *            état robot au moment du tir
     * @param target
     *            cible
     * @param yaw
     *            yaw utilisé (rad)
     * @param rpm
     *            rpm actuel (RPM)
     * @param hood
     *            hood actuel (deg)
     * @param span
     *            demi-largeur de la recherche en RPM
     * @return rpm raffiné (RPM) clampé min/max
     */
    private static double refineRpm(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d target,
            double yaw,
            double rpm,
            double hood,
            double span) {

        double lo = MathUtils.clamp(rpm - span, p.flywheelRpmMin(), p.flywheelRpmMax());
        double hi = MathUtils.clamp(rpm + span, p.flywheelRpmMin(), p.flywheelRpmMax());

        DoubleUnaryOperator f = (rr) -> ShooterLogic.simulateMissFast(p, robot, yaw, rr, hood, target).missM();
        MathUtils.Bracket br = MathUtils.bracketFromSamples(lo, hi, p.instantRpmSamples(), f);
        MathUtils.MinResult mr = MathUtils.goldenSectionMinimize(f, br.a(), br.b(), p.instantRpmGoldenIters());
        return MathUtils.clamp(mr.x(), p.flywheelRpmMin(), p.flywheelRpmMax());
    }

    /**
     * <p>
     * Raffine le hood en minimisant le miss sur un span donné.
     * </p>
     *
     * Note:
     * - Si solveForRpm est actif, on recalcule un RPM no-drag approximatif
     * pour que la recherche hood reste cohérente.
     *
     * @param p
     *            paramètres
     * @param robot
     *            état robot au moment du tir
     * @param target
     *            cible
     * @param yaw
     *            yaw utilisé (rad)
     * @param rpm
     *            rpm actuel (RPM)
     * @param hood
     *            hood actuel (deg)
     * @param span
     *            demi-largeur de la recherche hood (deg)
     * @return hood raffiné (deg) clampé min/max
     */
    private static double refineHood(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d target,
            double yaw,
            double rpm,
            double hood,
            double span) {

        double lo = MathUtils.clamp(hood - span, p.hoodMinDeg(), p.hoodMaxDeg());
        double hi = MathUtils.clamp(hood + span, p.hoodMinDeg(), p.hoodMaxDeg());

        final double hoodRange = Math.max(1e-6, p.hoodMaxDeg() - p.hoodMinDeg());
        final double hoodPreferSign = p.hoodPreferHigh() ? 1.0 : -1.0;

        DoubleUnaryOperator f = (hh) -> {
            double rUse = rpm;

            // Option: si on solve RPM, on approxime un RPM cohérent avec le nouveau hood
            if (p.solveForRpm()) {
                Double g = rpmGuessNoDrag(p, robot, hh, target);
                if (g != null)
                    rUse = 0.70 * g + 0.30 * rpm;
            }

            double miss = ShooterLogic.simulateMissFast(p, robot, yaw, rUse, hh, target).missM();

            // Poids pour préférer hood haut/bas si demandé
            if (p.hoodPreferWeight() > 0.0) {
                double hn = (hh - p.hoodMinDeg()) / hoodRange;
                miss -= hoodPreferSign * p.hoodPreferWeight() * hn;
            }
            return miss;
        };

        MathUtils.Bracket br = MathUtils.bracketFromSamples(lo, hi, p.instantHoodSamples(), f);
        MathUtils.MinResult mr = MathUtils.goldenSectionMinimize(f, br.a(), br.b(), p.instantHoodGoldenIters());
        return MathUtils.clamp(mr.x(), p.hoodMinDeg(), p.hoodMaxDeg());
    }

    /**
     * <p>
     * Guess un RPM sans drag (approx) pour aider à seed le solveur.
     * </p>
     *
     * But:
     * - Donner un point de départ realiste au solveur.
     * - Ça ne remplace pas la simulation avec drag, c'est juste un seed.
     *
     * Hypothèses:
     * - Pas de drag, juste gravité
     * - Tire à angle hoodDeg
     * - On résout une trajectoire parabolique simple pour atteindre (d, dz)
     *
     * @param p
     *            paramètres shooter
     * @param robot
     *            état robot (pour position muzzle)
     * @param hoodDeg
     *            angle de tir (deg)
     * @param targetXYZ
     *            cible sur le field
     * @return rpm estimé (RPM) ou null si la géométrie ne permet pas (ex: pas de solution)
     */
    private static Double rpmGuessNoDrag(
            Records.ShooterParams p,
            Records.RobotState robot,
            double hoodDeg,
            Translation3d targetXYZ) {

        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);

        double dx = targetXYZ.getX() - muzzle.getX();
        double dy = targetXYZ.getY() - muzzle.getY();
        double dz = targetXYZ.getZ() - muzzle.getZ();
        double d = Math.hypot(dx, dy);

        double theta = Math.toRadians(MathUtils.clamp(hoodDeg, p.hoodMinDeg(), p.hoodMaxDeg()));
        double ct = Math.cos(theta);
        double st = Math.sin(theta);
        if (ct < 1e-6)
            return null;

        double g = Math.max(p.g(), 1e-9);
        double tan = st / ct;

        // rhs = d*tan(theta) - dz doit être > 0 sinon pas de solution (ça viserait trop bas)
        double rhs = d * tan - dz;
        if (rhs <= 1e-9)
            return null;

        // Temps de vol approximatif
        double t = Math.sqrt(2.0 * rhs / g);
        if (!(t > 1e-6))
            return null;

        // v = d / (cos(theta) * t)
        double v = d / Math.max(ct * t, 1e-6);
        double rpm = ShooterLogic.exitSpeedToRpm(p, v);

        return MathUtils.clamp(rpm, p.flywheelRpmMin(), p.flywheelRpmMax());
    }

    /**
     * <p>
     * Seed yaw (premier guess) en tenant compte du mouvement du robot.
     * </p>
     *
     * Idée:
     * - On calcule yawField vers la cible (du muzzle)
     * - On approxime la vitesse horizontale de sortie vH
     * - On "ajuste" la direction pour que (v_robot + v_shot) pointe vers la cible
     * (approx: on corrige juste la partie horizontale)
     *
     * @param p
     *            paramètres shooter (pour muzzle + conversions)
     * @param robot
     *            état robot
     * @param targetXYZ
     *            cible field
     * @param rpm
     *            rpm utilisé pour estimer vH
     * @param hoodDeg
     *            hood utilisé pour estimer vH
     * @return yaw relatif robot (rad) clampé aux limites turret
     */
    private static double yawSeed(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d targetXYZ,
            double rpm,
            double hoodDeg) {

        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);
        double tx = targetXYZ.getX();
        double ty = targetXYZ.getY();

        // Direction directe vers la cible dans le field
        double yawField = Math.atan2(ty - muzzle.getY(), tx - muzzle.getX());

        // Vitesse de sortie (approx)
        double exit = ShooterLogic.rpmToExitSpeed(p, rpm);
        double elev = Math.toRadians(hoodDeg);
        double vH = exit * Math.cos(elev); // composante horizontale

        // Direction vers la cible
        double d0 = Math.cos(yawField);
        double d1 = Math.sin(yawField);

        // Vitesse robot
        double vr0 = robot.velXY().getX();
        double vr1 = robot.velXY().getY();

        // s = projection du robot sur la direction + vH
        // après on reconstruit un vecteur u = (v_total - v_robot) / vH
        double s = (vr0 * d0 + vr1 * d1) + vH;
        double ux = (s * d0 - vr0) / Math.max(vH, 1e-6);
        double uy = (s * d1 - vr1) / Math.max(vH, 1e-6);

        // Normalise pour récupérer l'angle
        double n = Math.hypot(ux, uy) + 1e-12;
        ux /= n;
        uy /= n;

        // yawU = direction ajustée dans le field
        double yawU = Math.atan2(uy, ux);

        // Convertit en yaw relatif au robot
        double yawRel = MathUtils.wrapRad(yawU - robot.yaw().getRadians());

        return ShooterLogic.clampTurretYaw(p, yawRel);
    }
}
