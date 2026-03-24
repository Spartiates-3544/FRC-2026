package frc.lib.archives;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.robot.Records;
import frc.lib.utils.MathUtils;

import java.util.function.DoubleUnaryOperator;

/**
 * Fast ballistic solver version for roboRIO.
 *
 * Main performance changes:
 * - Single pass only
 * - No widen passes
 * - Small local 3-point shrink search
 * - No second RPM refine after hood refine
 * - Hood only solved when RPM is near limits or RPM solving is off
 */
public final class BallisticSolver {
    private BallisticSolver() {
    }

    private static final double CONT_RPM_WEIGHT = 0.05;
    private static final double CONT_HOOD_WEIGHT = 2.50;
    private static final double CONT_YAW_WEIGHT = 1.00;

    private static final double RPM_NEAR_LIMIT_BAND = 150.0;
    private static final double HOOD_SPAN_SCALE = 0.45;

    public static Records.RobotState predictRobot(Records.RobotState r, double t) {
        Translation2d dp = r.velXY().times(t).plus(r.accelXY().times(0.5 * t * t));
        Translation2d pos = r.posXY().plus(dp);

        Translation2d vel = r.velXY().plus(r.accelXY().times(t));
        Rotation2d yaw = r.yaw().plus(Rotation2d.fromRadians(r.omegaRadS() * t));

        return new Records.RobotState(pos, yaw, vel, r.omegaRadS(), r.accelXY());
    }

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

    public static Records.SolveOutput solve(
            Records.ShooterParams p,
            Records.RobotState robotNow,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm) {

        double tFire = p.fireLatencyS();
        Records.ShotSolution sol;
        Records.Debug dbg;

        Records.RobotState pred = predictRobot(robotNow, tFire);
        SolvePoseOut poseOut = solveForPoseInstant(p, pred, actNow, targetXYZ, warm);
        PoseSolution ps = poseOut.sol();

        sol = new Records.ShotSolution(
                ps.ok(),
                ps.turretYawRelRad(),
                ps.flywheelRpm(),
                ps.hoodDeg(),
                tFire,
                ps.tofS(),
                ps.missM(),
                ps.info());

        double tReady = estimateTimeToReady(p, actNow, sol);
        tFire = p.fireLatencyS() + tReady;

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

        return new Records.SolveOutput(sol, dbg);
    }

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

        double missM() {
            return sol.missM();
        }

        double missXY() {
            return sol.missXY();
        }

        double missZ() {
            return sol.missZ();
        }

        boolean hit() {
            return sol.hit();
        }

        boolean sideEntry() {
            return sol.sideEntry();
        }
    }

    private static SolvePoseOut solveForPoseInstant(
            Records.ShooterParams p,
            Records.RobotState robot,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm) {

        double hoodGuessVar = (warm != null) ? warm.hoodDeg() : p.hoodFixedDeg();
        double rpmGuessVar = (warm != null) ? warm.flywheelRpm()
                : 0.5 * (p.flywheelRpmMin() + p.flywheelRpmMax());

        hoodGuessVar = MathUtils.clamp(hoodGuessVar, p.hoodMinDeg(), p.hoodMaxDeg());
        rpmGuessVar = MathUtils.clamp(rpmGuessVar, p.flywheelRpmMin(), p.flywheelRpmMax());

        if (!p.solveForHood()) {
            hoodGuessVar = MathUtils.clamp(p.hoodFixedDeg(), p.hoodMinDeg(), p.hoodMaxDeg());
        }

        if (p.solveForRpm()) {
            Double g = rpmGuessNoDrag(p, robot, hoodGuessVar, targetXYZ);
            if (g != null) {
                rpmGuessVar = 0.70 * g + 0.30 * rpmGuessVar;
            }
        }

        final double hoodGuess = hoodGuessVar;
        final double rpmGuess = rpmGuessVar;

        double seedYaw = yawSeed(p, robot, targetXYZ, rpmGuess, hoodGuess);

        if (warm != null) {
            seedYaw = ShooterLogic.clampTurretYaw(
                    p, MathUtils.wrapRad(0.78 * warm.turretYawRelRad() + 0.22 * seedYaw));
        }
        final double seedYawFinal = seedYaw;

        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);
        final double dist = Math.hypot(
                targetXYZ.getX() - muzzle.getX(),
                targetXYZ.getY() - muzzle.getY());

        final double yawWinDeg = MathUtils.clamp(
                6.5 * (4.0 / Math.max(dist, 1.0)),
                p.instantYawWinDegMin(),
                p.instantYawWinDegMax());
        final double yawWin = Math.toRadians(yawWinDeg);

        final double rpmSpan = MathUtils.clamp(
                p.instantRpmSpanMin() + 170.0 * dist,
                p.instantRpmSpanMin(),
                p.instantRpmSpanMax());

        final double hoodSpan = HOOD_SPAN_SCALE * MathUtils.clamp(
                3.2 * (4.0 / Math.max(dist, 1.0)),
                p.instantHoodSpanMin(),
                p.instantHoodSpanMax());

        final double warmYaw = (warm != null) ? warm.turretYawRelRad() : seedYawFinal;
        final double warmRpm = (warm != null) ? warm.flywheelRpm() : rpmGuess;
        final double warmHood = (warm != null) ? warm.hoodDeg() : hoodGuess;

        final double rpmJumpWindowS = 0.10;
        final double maxRpmJumpPerSolve = Math.max(250.0, p.flywheelAccelRpmS() * rpmJumpWindowS);

        final double hoodRange = Math.max(1e-6, (p.hoodMaxDeg() - p.hoodMinDeg()));
        final double hoodPreferSign = p.hoodPreferHigh() ? 1.0 : -1.0;

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

        java.util.function.Function<PoseSolution, Double> costFn = (s) -> {
            double dy = MathUtils.angleDiffRad(s.turretYawRelRad(), warmYaw);
            double dr = (s.flywheelRpm() - warmRpm) / 1000.0;
            double dh = (s.hoodDeg() - warmHood) / 10.0;

            double cont = (CONT_YAW_WEIGHT * dy * dy)
                    + (CONT_RPM_WEIGHT * dr * dr)
                    + (CONT_HOOD_WEIGHT * dh * dh);

            double c = s.missM() + p.continuityWeight() * cont;

            if (p.solveForHood() && p.hoodPreferWeight() > 0.0) {
                double hn = (s.hoodDeg() - p.hoodMinDeg()) / hoodRange;
                c -= hoodPreferSign * p.hoodPreferWeight() * hn;
            }
            return c;
        };

        java.util.function.BiPredicate<Double, Double> betterCost = (c, hoodVal) -> {
            if (c < best.cost - p.hoodTiebreakEps()) {
                return true;
            }

            if (Math.abs(c - best.cost) <= p.hoodTiebreakEps() && p.solveForHood()) {
                if (p.hoodPreferHigh()) {
                    return hoodVal > best.sol.hoodDeg();
                }
                return hoodVal < best.sol.hoodDeg();
            }
            return false;
        };

        double yawLocal = seedYawFinal;
        double rpmLocal = rpmGuess;
        double hoodLocal = hoodGuess;

        yawLocal = refineYaw(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, yawWin);

        if (p.solveForRpm()) {
            Double g = rpmGuessNoDrag(p, robot, hoodLocal, targetXYZ);
            if (g != null) {
                rpmLocal = 0.80 * g + 0.20 * rpmLocal;
            }

            rpmLocal = refineRpm(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, rpmSpan);
            rpmLocal = MathUtils.clamp(
                    rpmLocal,
                    warmRpm - maxRpmJumpPerSolve,
                    warmRpm + maxRpmJumpPerSolve);
        }

        Records.SimResult preHoodSim = ShooterLogic.simulateMissFast(
                p,
                robot,
                ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(yawLocal)),
                MathUtils.clamp(rpmLocal, p.flywheelRpmMin(), p.flywheelRpmMax()),
                MathUtils.clamp(hoodLocal, p.hoodMinDeg(), p.hoodMaxDeg()),
                targetXYZ);

        boolean needHoodSolve = p.solveForHood()
                && (!p.solveForRpm()
                        || rpmLocal >= p.flywheelRpmMax() - RPM_NEAR_LIMIT_BAND
                        || rpmLocal <= p.flywheelRpmMin() + RPM_NEAR_LIMIT_BAND)
                && preHoodSim.missM() > (0.4 * p.hitRadiusM());

        if (needHoodSolve) {
            hoodLocal = refineHood(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, hoodSpan);
            hoodLocal = MathUtils.clamp(hoodLocal, p.hoodMinDeg(), p.hoodMaxDeg());
        }

        double yawRaw = yawLocal;
        double rpmRaw = rpmLocal;
        double hoodRaw = hoodLocal;

        double yawCmd = ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(yawRaw));
        double hoodCmd = MathUtils.clamp(hoodRaw, p.hoodMinDeg(), p.hoodMaxDeg());
        double rpmCmd = MathUtils.clamp(rpmRaw, p.flywheelRpmMin(), p.flywheelRpmMax());

        boolean turretClamped = Math.abs(MathUtils.wrapRad(yawCmd - yawRaw)) > Math.toRadians(0.02);
        boolean hoodClamped = Math.abs(hoodCmd - hoodRaw) > 1e-6;
        boolean rpmClamped = Math.abs(rpmCmd - rpmRaw) > 1e-6;

        Records.SimResult sim = ShooterLogic.simulateMissFast(p, robot, yawCmd, rpmCmd, hoodCmd, targetXYZ);

        PoseSolution cand = new PoseSolution(
                sim.hit() || sim.missM() <= p.hitRadiusM(),
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

    private static double refineYaw(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d target,
            double yaw,
            double rpm,
            double hood,
            double yawWin) {

        double min = yaw - yawWin;
        double max = yaw + yawWin;

        DoubleUnaryOperator f = (yyU) -> {
            double yy = ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(yyU));
            return ShooterLogic.simulateMissFast(p, robot, yy, rpm, hood, target).missM();
        };

        double out = localBest(f, yaw, min, max, yawWin * 0.5, 2);
        return ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(out));
    }

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
        return localBest(f, rpm, lo, hi, Math.max(25.0, span * 0.5), 2);
    }

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

        DoubleUnaryOperator f = (hh) -> {
            double rUse = rpm;
            if (p.solveForRpm()) {
                Double g = rpmGuessNoDrag(p, robot, hh, target);
                if (g != null) {
                    rUse = 0.70 * g + 0.30 * rpm;
                }
            }
            return ShooterLogic.simulateMissFast(p, robot, yaw, rUse, hh, target).missM();
        };

        return localBest(f, hood, lo, hi, Math.max(0.25, span * 0.5), 2);
    }

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
        if (ct < 1e-6) {
            return null;
        }

        double g = Math.max(p.g(), 1e-9);
        double tan = st / ct;

        double rhs = d * tan - dz;
        if (rhs <= 1e-9) {
            return null;
        }

        double t = Math.sqrt(2.0 * rhs / g);
        if (!(t > 1e-6)) {
            return null;
        }

        double v = d / Math.max(ct * t, 1e-6);
        double rpm = ShooterLogic.exitSpeedToRpm(p, v);

        return MathUtils.clamp(rpm, p.flywheelRpmMin(), p.flywheelRpmMax());
    }

    private static double localBest(
            DoubleUnaryOperator f,
            double center,
            double min,
            double max,
            double span,
            int iters) {

        double x = MathUtils.clamp(center, min, max);
        double s = Math.max(1e-6, span);

        for (int i = 0; i < iters; i++) {
            double xl = MathUtils.clamp(x - s, min, max);
            double xc = x;
            double xr = MathUtils.clamp(x + s, min, max);

            double fl = f.applyAsDouble(xl);
            double fc = f.applyAsDouble(xc);
            double fr = f.applyAsDouble(xr);

            if (fl <= fc && fl <= fr) {
                x = xl;
            } else if (fr <= fc && fr <= fl) {
                x = xr;
            } else {
                x = xc;
            }

            s *= 0.5;
        }

        return MathUtils.clamp(x, min, max);
    }

    private static double yawSeed(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d targetXYZ,
            double rpm,
            double hoodDeg) {

        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);
        double tx = targetXYZ.getX();
        double ty = targetXYZ.getY();

        double yawField = Math.atan2(ty - muzzle.getY(), tx - muzzle.getX());

        double exit = ShooterLogic.rpmToExitSpeed(p, rpm);
        double elev = Math.toRadians(hoodDeg);
        double vH = exit * Math.cos(elev);

        double d0 = Math.cos(yawField);
        double d1 = Math.sin(yawField);

        double vr0 = robot.velXY().getX();
        double vr1 = robot.velXY().getY();

        double s = (vr0 * d0 + vr1 * d1) + vH;
        double ux = (s * d0 - vr0) / Math.max(vH, 1e-6);
        double uy = (s * d1 - vr1) / Math.max(vH, 1e-6);

        double n = Math.hypot(ux, uy) + 1e-12;
        ux /= n;
        uy /= n;

        double yawU = Math.atan2(uy, ux);
        double yawRel = MathUtils.wrapRad(yawU - robot.yaw().getRadians());

        return ShooterLogic.clampTurretYaw(p, yawRel);
    }
}