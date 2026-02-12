// frc/mentor/logic/BallisticSolver.java
package frc.mentor.logic;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.DoubleUnaryOperator;

import frc.mentor.robot.Records;

public final class BallisticSolver {
    private BallisticSolver() {
    }

    public static Records.RobotState predictRobot(Records.RobotState r, double t) {
        Translation2d dp = r.velXY().times(t).plus(r.accelXY().times(0.5 * t * t));
        Translation2d pos = r.posXY().plus(dp);

        // Update velocity too (constant accel model)
        Translation2d vel = r.velXY().plus(r.accelXY().times(t));

        Rotation2d yaw = r.yaw().plus(Rotation2d.fromRadians(r.omegaRadS() * t));
        return new Records.RobotState(pos, yaw, vel, r.omegaRadS(), r.accelXY());
    }

    public static double estimateTimeToReady(Records.ShooterParams p, Records.ActuatorState act,
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
        Records.ShotSolution sol = null;
        Records.Debug dbg = null;

        for (int i = 0; i < 2; i++) {
            Records.RobotState pred = predictRobot(robotNow, tFire);
            SolvePoseOut poseOut = solveForPoseInstant(p, pred, targetXYZ, sol == null ? warm : sol);

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

            // Debug should match FINAL tFire (recompute pred)
            Records.RobotState predFinal = predictRobot(robotNow, tFire);

            dbg = new Records.Debug(
                    tReady,
                    tFire,
                    predFinal,
                    poseOut.seedYaw(),
                    poseOut.distXY(),
                    ps.ok(),
                    ps.missM(),
                    ShooterLogic.rpmToExitSpeed(p, sol.flywheelRpm()));
        }

        return new Records.SolveOutput(sol, dbg);
    }

    private static record PoseSolution(
            boolean ok,
            double turretYawRelRad,
            double flywheelRpm,
            double hoodDeg,
            double tofS,
            double missM,
            String info) {
    }

    private static record SolvePoseOut(PoseSolution sol, double seedYaw, double distXY) {
    }

    private static SolvePoseOut solveForPoseInstant(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d targetXYZ,
            Records.ShotSolution warm) {

        double hood = (warm != null) ? warm.hoodDeg() : p.hoodFixedDeg();
        double rpm = (warm != null) ? warm.flywheelRpm() : 0.5 * (p.flywheelRpmMin() + p.flywheelRpmMax());

        hood = MathUtils.clamp(hood, p.hoodMinDeg(), p.hoodMaxDeg());
        rpm = MathUtils.clamp(rpm, p.flywheelRpmMin(), p.flywheelRpmMax());

        if (!p.solveForHood())
            hood = MathUtils.clamp(p.hoodFixedDeg(), p.hoodMinDeg(), p.hoodMaxDeg());

        if (p.solveForRpm()) {
            Double g = rpmGuessNoDrag(p, robot, hood, targetXYZ);
            if (g != null)
                rpm = 0.70 * g + 0.30 * rpm;
        }

        double seedYaw = yawSeed(p, robot, targetXYZ, rpm, hood);
        if (warm != null) {
            seedYaw = ShooterLogic.clampTurretYaw(
                    p, MathUtils.wrapRad(0.78 * warm.turretYawRelRad() + 0.22 * seedYaw));
        }

        final double seedYawFinal = seedYaw;
        final double hoodFinal = hood;
        final double rpmFinal = rpm;

        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);
        double dist = Math.hypot(targetXYZ.getX() - muzzle.getX(), targetXYZ.getY() - muzzle.getY());

        double yawWinDeg = MathUtils.clamp(
                6.5 * (4.0 / Math.max(dist, 1.0)),
                p.instantYawWinDegMin(),
                p.instantYawWinDegMax());
        double yawWin = Math.toRadians(yawWinDeg);

        double rpmSpan = MathUtils.clamp(
                p.instantRpmSpanMin() + 170.0 * dist,
                p.instantRpmSpanMin(),
                p.instantRpmSpanMax());

        double hoodSpan = MathUtils.clamp(
                3.2 * (4.0 / Math.max(dist, 1.0)),
                p.instantHoodSpanMin(),
                p.instantHoodSpanMax());

        double warmYaw = (warm != null) ? warm.turretYawRelRad() : seedYawFinal;
        double warmRpm = (warm != null) ? warm.flywheelRpm() : rpmFinal;
        double warmHood = (warm != null) ? warm.hoodDeg() : hoodFinal;

        double hoodRange = Math.max(1e-6, (p.hoodMaxDeg() - p.hoodMinDeg()));
        final double hoodPreferSign = p.hoodPreferHigh() ? 1.0 : -1.0;

        class Best {
            PoseSolution sol = new PoseSolution(false, seedYawFinal, rpmFinal, hoodFinal, 0.0, 1e9, "instant");
            double cost = 1e30;
        }
        Best best = new Best();

        java.util.function.Function<PoseSolution, Double> costFn = (s) -> {
            double dy = MathUtils.angleDiffRad(s.turretYawRelRad(), warmYaw);
            double dr = (s.flywheelRpm() - warmRpm) / 1000.0;
            double dh = (s.hoodDeg() - warmHood) / 10.0;

            double c = s.missM() + p.continuityWeight() * (dy * dy + 0.20 * dr * dr + 0.30 * dh * dh);

            // Apply preference with correct direction
            if (p.solveForHood() && p.hoodPreferWeight() > 0.0) {
                double hn = (s.hoodDeg() - p.hoodMinDeg()) / hoodRange; // 0..1
                c -= hoodPreferSign * p.hoodPreferWeight() * hn;
            }
            return c;
        };

        java.util.function.BiPredicate<Double, Double> betterCost = (c, hoodVal) -> {
            if (c < best.cost - p.hoodTiebreakEps())
                return true;
            if (Math.abs(c - best.cost) <= p.hoodTiebreakEps() && p.solveForHood() && p.hoodPreferHigh()) {
                return hoodVal > best.sol.hoodDeg();
            }
            return false;
        };

        int passes = Math.max(1, p.instantIters());
        double yawLocal = seedYawFinal;
        double rpmLocal = rpmFinal;
        double hoodLocal = hoodFinal;

        double yawWinLocal = yawWin;
        double rpmSpanLocal = rpmSpan;
        double hoodSpanLocal = hoodSpan;

        int widenPasses = (p.instantWidenIfMiss() ? Math.max(0, p.instantWidenPasses()) : 0);

        for (int widen = 0; widen <= widenPasses; widen++) {
            for (int it = 0; it < passes; it++) {
                yawLocal = refineYaw(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, yawWinLocal);

                if (p.solveForRpm()) {
                    Double g = rpmGuessNoDrag(p, robot, hoodLocal, targetXYZ);
                    if (g != null)
                        rpmLocal = 0.80 * g + 0.20 * rpmLocal;
                    rpmLocal = refineRpm(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, rpmSpanLocal);
                }

                if (p.solveForHood()) {
                    hoodLocal = refineHood(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, hoodSpanLocal);

                    if (p.solveForRpm()) {
                        Double g2 = rpmGuessNoDrag(p, robot, hoodLocal, targetXYZ);
                        if (g2 != null)
                            rpmLocal = 0.70 * g2 + 0.30 * rpmLocal;
                        rpmLocal = refineRpm(p, robot, targetXYZ, yawLocal, rpmLocal, hoodLocal, 0.55 * rpmSpanLocal);
                    }
                }
            }

            Records.SimResult sim = ShooterLogic.simulateMissFast(p, robot, yawLocal, rpmLocal, hoodLocal, targetXYZ);
            PoseSolution cand = new PoseSolution(
                    sim.hit() || sim.missM() <= p.hitRadiusM(),
                    yawLocal,
                    rpmLocal,
                    hoodLocal,
                    sim.t(),
                    sim.missM(),
                    sim.hit() ? "hit" : "instant");

            double c = costFn.apply(cand);
            if (betterCost.test(c, cand.hoodDeg())) {
                best.cost = c;
                best.sol = cand;
            }

            if (widen < widenPasses && best.sol.missM() > p.hitRadiusM()) {
                yawWinLocal = Math.min(yawWinLocal * p.instantWidenYawMul(), Math.toRadians(p.instantYawWinDegMax()));
                rpmSpanLocal = Math.min(rpmSpanLocal * p.instantWidenRpmMul(), p.instantRpmSpanMax());
            }
        }

        if (p.solveForHood() && p.hoodPreferHigh() && p.hoodPushupSteps() > 0) {
            PoseSolution b = best.sol;
            double baseMiss = b.missM();
            double yawB = b.turretYawRelRad();
            double rpmB = b.flywheelRpm();
            double hoodB = b.hoodDeg();

            for (int s = 1; s <= p.hoodPushupSteps(); s++) {
                double hTry = MathUtils.clamp(hoodB + s, p.hoodMinDeg(), p.hoodMaxDeg());
                if (hTry <= hoodB + 1e-6)
                    continue;

                double yTry = refineYaw(
                        p, robot, targetXYZ, yawB, rpmB, hTry,
                        Math.min(yawWin, Math.toRadians(1.2)));

                double rTry = rpmB;
                if (p.solveForRpm()) {
                    Double g = rpmGuessNoDrag(p, robot, hTry, targetXYZ);
                    if (g != null)
                        rTry = 0.75 * g + 0.25 * rTry;
                    rTry = refineRpm(p, robot, targetXYZ, yTry, rTry, hTry, Math.min(rpmSpan, 700.0));
                }

                Records.SimResult sim = ShooterLogic.simulateMissFast(p, robot, yTry, rTry, hTry, targetXYZ);
                if (sim.missM() <= baseMiss + p.hoodPushupEpsM()) {
                    PoseSolution cand = new PoseSolution(
                            sim.hit() || sim.missM() <= p.hitRadiusM(),
                            yTry,
                            rTry,
                            hTry,
                            sim.t(),
                            sim.missM(),
                            sim.hit() ? "hit" : "pushup");

                    double c = costFn.apply(cand);
                    if (betterCost.test(c, hTry)) {
                        best.cost = c;
                        best.sol = cand;
                        baseMiss = cand.missM();
                        yawB = cand.turretYawRelRad();
                        rpmB = cand.flywheelRpm();
                        hoodB = cand.hoodDeg();
                    }
                }
            }
        }

        return new SolvePoseOut(best.sol, seedYawFinal, dist);
    }

    private static double refineYaw(
            Records.ShooterParams p,
            Records.RobotState robot,
            Translation3d target,
            double yaw,
            double rpm,
            double hood,
            double yawWin) {

        // Use unwrapped interval around yaw to avoid crossing +/-pi issues.
        double loU = yaw - yawWin;
        double hiU = yaw + yawWin;

        DoubleUnaryOperator f = (yyU) -> {
            double yy = ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(yyU));
            return ShooterLogic.simulateMissFast(p, robot, yy, rpm, hood, target).missM();
        };

        MathUtils.Bracket br = MathUtils.bracketFromSamples(loU, hiU, p.instantYawSamples(), f);
        MathUtils.MinResult mr = MathUtils.goldenSectionMinimize(f, br.a(), br.b(), p.instantYawGoldenIters());
        return ShooterLogic.clampTurretYaw(p, MathUtils.wrapRad(mr.x()));
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
        MathUtils.Bracket br = MathUtils.bracketFromSamples(lo, hi, p.instantRpmSamples(), f);
        MathUtils.MinResult mr = MathUtils.goldenSectionMinimize(f, br.a(), br.b(), p.instantRpmGoldenIters());
        return MathUtils.clamp(mr.x(), p.flywheelRpmMin(), p.flywheelRpmMax());
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

        double hoodRange = Math.max(1e-6, p.hoodMaxDeg() - p.hoodMinDeg());
        final double hoodPreferSign = p.hoodPreferHigh() ? 1.0 : -1.0;

        DoubleUnaryOperator f = (hh) -> {
            double rUse = rpm;
            if (p.solveForRpm()) {
                Double g = rpmGuessNoDrag(p, robot, hh, target);
                if (g != null)
                    rUse = 0.70 * g + 0.30 * rpm;
            }

            double miss = ShooterLogic.simulateMissFast(p, robot, yaw, rUse, hh, target).missM();

            // Apply preference with correct direction
            if (p.hoodPreferWeight() > 0.0) {
                double hn = (hh - p.hoodMinDeg()) / hoodRange; // 0..1
                miss -= hoodPreferSign * p.hoodPreferWeight() * hn;
            }
            return miss;
        };

        MathUtils.Bracket br = MathUtils.bracketFromSamples(lo, hi, p.instantHoodSamples(), f);
        MathUtils.MinResult mr = MathUtils.goldenSectionMinimize(f, br.a(), br.b(), p.instantHoodGoldenIters());
        return MathUtils.clamp(mr.x(), p.hoodMinDeg(), p.hoodMaxDeg());
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
        if (ct < 1e-6)
            return null;

        double a = 0.5 * p.g();
        double b = -d * (st / ct);
        double c = dz;

        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0)
            return null;

        double sd = Math.sqrt(Math.max(0.0, disc));
        double t1 = (-b - sd) / (2.0 * a);
        double t2 = (-b + sd) / (2.0 * a);

        double t;
        if (t1 > 1e-4 && t2 > 1e-4)
            t = Math.min(t1, t2);
        else if (t1 > 1e-4)
            t = t1;
        else if (t2 > 1e-4)
            t = t2;
        else
            return null;

        double v = d / Math.max(ct * t, 1e-6);
        double rpm = ShooterLogic.exitSpeedToRpm(p, v);
        return MathUtils.clamp(rpm, p.flywheelRpmMin(), p.flywheelRpmMax());
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
