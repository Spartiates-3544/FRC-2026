package frc.lib.logic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.robot.Records;
import frc.lib.utils.MathUtils;

public final class FastShooterSolver {

    private FastShooterSolver() {}

    public static final class DistanceRpmTable {
        private final double[] distanceM;
        private final double[] rpm;

        public DistanceRpmTable(double[] distanceM, double[] rpm) {
            if (distanceM == null || rpm == null || distanceM.length == 0 || rpm.length == 0) {
                throw new IllegalArgumentException("Distance/RPM table cannot be empty");
            }
            if (distanceM.length != rpm.length) {
                throw new IllegalArgumentException("Distance/RPM table sizes must match");
            }
            this.distanceM = distanceM.clone();
            this.rpm = rpm.clone();
        }

        public double lookupRpm(double distance) {
            if (distance <= distanceM[0]) {
                return rpm[0];
            }
            int last = distanceM.length - 1;
            if (distance >= distanceM[last]) {
                return rpm[last];
            }

            for (int i = 0; i < last; i++) {
                double x0 = distanceM[i];
                double x1 = distanceM[i + 1];
                if (distance >= x0 && distance <= x1) {
                    double u = (distance - x0) / Math.max(1e-9, x1 - x0);
                    return rpm[i] + u * (rpm[i + 1] - rpm[i]);
                }
            }

            return rpm[last];
        }
    }

    public static Records.ShotSolution solve(
            Records.ShooterParams p,
            Records.RobotState robot,
            Records.ActuatorState act,
            Translation3d targetXYZ,
            DistanceRpmTable rpmTable,
            double fixedHoodDeg) {

        if (robot == null || act == null || targetXYZ == null) {
            return invalid(act, 0.0, fixedHoodDeg, "missing_inputs");
        }

        Translation3d muzzle = ShooterLogic.muzzlePositionField(p, robot);

        double dx = targetXYZ.getX() - muzzle.getX();
        double dy = targetXYZ.getY() - muzzle.getY();
        double dz = targetXYZ.getZ() - muzzle.getZ();

        double distanceXY = Math.hypot(dx, dy);

        double rpm = rpmTable.lookupRpm(distanceXY);
        rpm = MathUtil.clamp(rpm, p.flywheelRpmMin(), p.flywheelRpmMax());

        double hoodDeg = MathUtil.clamp(fixedHoodDeg, p.hoodMinDeg(), p.hoodMaxDeg());
        double hoodRad = Math.toRadians(hoodDeg);

        double exitSpeed = ShooterLogic.rpmToExitSpeed(p, rpm);
        double horizShotSpeed = exitSpeed * Math.cos(hoodRad);
        double vertShotSpeed = exitSpeed * Math.sin(hoodRad);

        if (horizShotSpeed <= 1e-6) {
            return invalid(act, rpm, hoodDeg, "bad_horizontal_speed");
        }

        double rvx = robot.velXY().getX();
        double rvy = robot.velXY().getY();

        double tof = solveHorizontalInterceptTime(dx, dy, rvx, rvy, horizShotSpeed);
        if (!(tof > 0.0) || !Double.isFinite(tof)) {
            return invalid(act, rpm, hoodDeg, "no_intercept");
        }

        double aimX = (dx / tof) - rvx;
        double aimY = (dy / tof) - rvy;

        double aimNorm = Math.hypot(aimX, aimY);
        if (aimNorm < 1e-9) {
            return invalid(act, rpm, hoodDeg, "bad_aim_vector");
        }

        double yawField = Math.atan2(aimY, aimX);
        double turretYawRelRad = MathUtils.wrapRad(yawField - robot.yaw().getRadians());
        turretYawRelRad = ShooterLogic.clampTurretYaw(p, turretYawRelRad);

        boolean blind = ShooterLogic.isTurretBlindSpot(p, turretYawRelRad);
        if (blind) {
            return new Records.ShotSolution(
                    false,
                    act.turretYawRelRad(),
                    rpm,
                    hoodDeg,
                    0.0,
                    tof,
                    999.0,
                    "blind_hold");
        }

        double predictedZ = vertShotSpeed * tof - 0.5 * p.g() * tof * tof;
        double zError = dz - predictedZ;
        double missM = Math.abs(zError);

        boolean ok = missM <= p.hitRadiusM();

        return new Records.ShotSolution(
                ok,
                turretYawRelRad,
                rpm,
                hoodDeg,
                0.0,
                tof,
                missM,
                ok ? "fast_lead_ok" : "fast_lead_vertical_miss");
    }

    private static Records.ShotSolution invalid(
            Records.ActuatorState act,
            double rpm,
            double hoodDeg,
            String info) {

        double holdYaw = act != null ? act.turretYawRelRad() : 0.0;

        return new Records.ShotSolution(
                false,
                holdYaw,
                rpm,
                hoodDeg,
                0.0,
                0.0,
                999.0,
                info);
    }

    /**
     * Solve |r / t - vRobot| = shotHorizontalSpeed for positive t.
     */
    private static double solveHorizontalInterceptTime(
            double rx,
            double ry,
            double vx,
            double vy,
            double shotHorizontalSpeed) {

        double a = (vx * vx + vy * vy) - (shotHorizontalSpeed * shotHorizontalSpeed);
        double b = -2.0 * (rx * vx + ry * vy);
        double c = rx * rx + ry * ry;

        if (Math.abs(a) < 1e-9) {
            if (Math.abs(b) < 1e-9) {
                return Double.NaN;
            }
            double t = -c / b;
            return t > 0.0 ? t : Double.NaN;
        }

        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0) {
            return Double.NaN;
        }

        double sqrtDisc = Math.sqrt(disc);
        double t1 = (-b - sqrtDisc) / (2.0 * a);
        double t2 = (-b + sqrtDisc) / (2.0 * a);

        double best = Double.NaN;
        if (t1 > 0.0) {
            best = t1;
        }
        if (t2 > 0.0 && (!Double.isFinite(best) || t2 < best)) {
            best = t2;
        }
        return best;
    }
}