package frc.lib.logic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.archives.ShooterLogic;
import frc.lib.robot.Records;
import frc.lib.utils.MathUtils;

public final class FastShooterSolver {

    /**
     * Utility class only.
     * No instances should ever exist.
     */
    private FastShooterSolver() {
    }

    /**
     * Small lookup table used to estimate required flywheel RPM from horizontal distance.
     *
     * This is a simple piecewise-linear interpolation table:
     * - if the distance is below the first point, clamp to the first RPM
     * - if the distance is above the last point, clamp to the last RPM
     * - otherwise interpolate between the 2 surrounding points
     */
    public static final class DistanceRpmTable {
        private final double[] distancesMeters;
        private final double[] rpms;
        private final double[] angles;

        public DistanceRpmTable(double[] distancesMeters, double[] rpms, double[] angles) {
            if (distancesMeters == null || rpms == null || angles == null
                    || distancesMeters.length == 0 || rpms.length == 0 || angles.length == 0) {
                throw new IllegalArgumentException("Distance/RPM table cannot be empty");
            }

            if ((distancesMeters.length != rpms.length) && (distancesMeters.length != angles.length)) {
                throw new IllegalArgumentException("Distance/RPM table sizes must match");
            }

            this.distancesMeters = distancesMeters.clone();
            this.rpms = rpms.clone();
            this.angles = angles.clone();
        }

        /**
         * Returns the interpolated RPM for a given horizontal distance.
         */
        public double lookupRpm(double distanceMeters) {
            // Clamp below table range
            if (distanceMeters <= distancesMeters[0]) {
                return rpms[0];
            }

            int lastIndex = distancesMeters.length - 1;

            // Clamp above table range
            if (distanceMeters >= distancesMeters[lastIndex]) {
                return rpms[lastIndex];
            }

            // Find the segment that contains the requested distance
            for (int i = 0; i < lastIndex; i++) {
                double x0 = distancesMeters[i];
                double x1 = distancesMeters[i + 1];

                if (distanceMeters >= x0 && distanceMeters <= x1) {
                    // Linear interpolation factor from 0 to 1
                    double blend = (distanceMeters - x0) / Math.max(1e-9, x1 - x0);

                    // Interpolate RPM between the two points
                    return rpms[i] + (blend * (rpms[i + 1] - rpms[i]));
                }
            }

            // Should never happen if the table is valid, but keep a safe fallback
            return rpms[lastIndex];
        }

        /**
         * Returns the interpolated angle for a given horizontal distance.
         */
        public double lookupAngle(double distanceMeters) {
            // Clamp below table range
            if (distanceMeters <= distancesMeters[0]) {
                return angles[0];
            }

            int lastIndex = distancesMeters.length - 1;

            // Clamp above table range
            if (distanceMeters >= distancesMeters[lastIndex]) {
                return angles[lastIndex];
            }

            // Find the segment that contains the requested distance
            for (int i = 0; i < lastIndex; i++) {
                double x0 = distancesMeters[i];
                double x1 = distancesMeters[i + 1];

                if (distanceMeters >= x0 && distanceMeters <= x1) {
                    // Linear interpolation factor from 0 to 1
                    double blend = (distanceMeters - x0) / Math.max(1e-9, x1 - x0);

                    // Interpolate RPM between the two points
                    return angles[i] + (blend * (angles[i + 1] - angles[i]));
                }
            }

            // Should never happen if the table is valid, but keep a safe fallback
            return angles[lastIndex];
        }
    }

    /**
     * Fast lead solver:
     *
     * Goal:
     * - pick flywheel RPM from distance
     * - use a fixed hood angle
     * - solve horizontal lead analytically
     * - estimate vertical miss
     * - return a shot solution quickly
     *
     * This is intentionally simpler and faster than a full iterative ballistic optimizer.
     */
    public static Records.ShotSolution solve(
            Records.ShooterParams params,
            Records.RobotState robotState,
            Records.ActuatorState actuatorState,
            Translation3d targetPosition,
            DistanceRpmTable rpmTable,
            double fixedHoodDeg) {

        // Basic input validation
        if (robotState == null || actuatorState == null || targetPosition == null) {
            return invalidSolution(actuatorState, 0.0, fixedHoodDeg, "missing_inputs");
        }

        // ---------------------------------------------------------------------
        // 1) Compute muzzle position in field coordinates
        // ---------------------------------------------------------------------
        Translation3d muzzlePosition = ShooterLogic.muzzlePositionField(params, robotState);

        // Predict muzzle position at fire time (latency compensation)
        double fireLat = params.fireLatencyS();
        double firedMuzzleX = muzzlePosition.getX() + robotState.velXY().getX() * fireLat;
        double firedMuzzleY = muzzlePosition.getY() + robotState.velXY().getY() * fireLat;

        // Vector from predicted fire position to target in field frame
        double deltaX = targetPosition.getX() - firedMuzzleX;
        double deltaY = targetPosition.getY() - firedMuzzleY;
        double deltaZ = targetPosition.getZ() - muzzlePosition.getZ();

        // Horizontal distance only (XY plane)
        double horizontalDistanceMeters = Math.hypot(deltaX, deltaY);

        // ---------------------------------------------------------------------
        // 2) Select shooter RPM and hood angle
        // ---------------------------------------------------------------------
        double flywheelRpm = rpmTable.lookupRpm(horizontalDistanceMeters);
        flywheelRpm = MathUtil.clamp(
                flywheelRpm,
                params.flywheelRpmMin(),
                params.flywheelRpmMax());

        double hoodAngleDeg = rpmTable.lookupAngle(horizontalDistanceMeters);
        hoodAngleDeg = MathUtil.clamp(
                hoodAngleDeg,
                params.hoodMinDeg(),
                params.hoodMaxDeg());

        double hoodAngleRad = Math.toRadians(hoodAngleDeg);

        // ---------------------------------------------------------------------
        // 3) Convert shot settings into projectile exit velocity
        // ---------------------------------------------------------------------
        double exitSpeedMetersPerSec = ShooterLogic.rpmToExitSpeed(params, flywheelRpm);

        // Split exit speed into horizontal and vertical components
        double horizontalShotSpeed = exitSpeedMetersPerSec * Math.cos(hoodAngleRad);

        // If horizontal speed is basically zero, there is no valid intercept
        if (horizontalShotSpeed <= 1e-6) {
            return invalidSolution(actuatorState, flywheelRpm, hoodAngleDeg, "bad_horizontal_speed");
        }

        // ---------------------------------------------------------------------
        // 4) Read robot translational velocity in field frame
        // ---------------------------------------------------------------------
        double robotVelX = robotState.velXY().getX();
        double robotVelY = robotState.velXY().getY();

        // ---------------------------------------------------------------------
        // 5) Solve time-of-flight for horizontal intercept
        // ---------------------------------------------------------------------
        double timeOfFlightSec = solveHorizontalInterceptTime(
                deltaX,
                deltaY,
                robotVelX,
                robotVelY,
                horizontalShotSpeed);

        if (!(timeOfFlightSec > 0.0) || !Double.isFinite(timeOfFlightSec)) {
            return invalidSolution(actuatorState, flywheelRpm, hoodAngleDeg, "no_intercept");
        }

        // ---------------------------------------------------------------------
        // 6) Compute the required horizontal aim direction
        // ---------------------------------------------------------------------
        // We want the projectile's horizontal velocity to cancel robot motion
        // and still arrive at the target in the solved time of flight.
        double requiredAimX = (deltaX / timeOfFlightSec) - robotVelX;
        double requiredAimY = (deltaY / timeOfFlightSec) - robotVelY;

        double aimVectorMagnitude = Math.hypot(requiredAimX, requiredAimY);
        if (aimVectorMagnitude < 1e-9) {
            return invalidSolution(actuatorState, flywheelRpm, hoodAngleDeg, "bad_aim_vector");
        }

        // Field-relative yaw needed to point along the aim vector
        double aimYawFieldRad = Math.atan2(requiredAimY, requiredAimX);

        // Convert to turret-relative yaw
        double turretYawRelativeRad = MathUtils.wrapRad(
                aimYawFieldRad - robotState.yaw().getRadians());

        // Clamp to turret mechanical limits
        turretYawRelativeRad = ShooterLogic.clampTurretYaw(params, turretYawRelativeRad);

        // ---------------------------------------------------------------------
        // 7) Reject blind-spot solutions
        // ---------------------------------------------------------------------
        boolean isBlindSpot = ShooterLogic.isTurretBlindSpot(params, turretYawRelativeRad);
        if (isBlindSpot) {
            // Hold current turret angle instead of commanding a bad solution
            return new Records.ShotSolution(
                    false,
                    actuatorState.turretYawRelRad(),
                    flywheelRpm,
                    hoodAngleDeg,
                    0.0,
                    timeOfFlightSec,
                    999.0,
                    "blind_hold");
        }

        // ---------------------------------------------------------------------
        // 8) Adjust RPM for robot velocity (moving shots)
        //
        // Yaw and TOF are fully solved above. RPM correction is applied last so
        // it cannot affect the aim direction. We shift the effective lookup
        // distance by the robot's radial velocity × TOF: moving toward the goal
        // is like shooting from closer (less RPM needed), moving away needs more.
        // Static shots: radialVelocity ≈ 0 → no change.
        // ---------------------------------------------------------------------
        double distUnitX = deltaX / Math.max(1e-9, horizontalDistanceMeters);
        double distUnitY = deltaY / Math.max(1e-9, horizontalDistanceMeters);
        double radialVelocity = robotVelX * distUnitX + robotVelY * distUnitY;
        double effectiveDistance = MathUtil.clamp(
                horizontalDistanceMeters - radialVelocity * timeOfFlightSec,
                0.1, Double.MAX_VALUE);
        double adjustedRpm = flywheelRpm + (rpmTable.lookupRpm(effectiveDistance) - rpmTable.lookupRpm(horizontalDistanceMeters));
        adjustedRpm = MathUtil.clamp(adjustedRpm, params.flywheelRpmMin(), params.flywheelRpmMax());

        double adjustedExitSpeed = ShooterLogic.rpmToExitSpeed(params, adjustedRpm);
        double adjustedVerticalSpeed = adjustedExitSpeed * Math.sin(hoodAngleRad);

        // ---------------------------------------------------------------------
        // 9) Predict vertical travel and estimate miss distance
        // ---------------------------------------------------------------------
        double predictedVerticalTravel = adjustedVerticalSpeed * timeOfFlightSec
                - 0.5 * params.g() * timeOfFlightSec * timeOfFlightSec;

        double verticalErrorMeters = deltaZ - predictedVerticalTravel;
        double missDistanceMeters = Math.abs(verticalErrorMeters);

        boolean isHit = missDistanceMeters <= params.hitRadiusM();

        // ---------------------------------------------------------------------
        // 10) Return final shot solution
        // ---------------------------------------------------------------------
        return new Records.ShotSolution(
                isHit,
                turretYawRelativeRad,
                adjustedRpm,
                hoodAngleDeg,
                0.0,
                timeOfFlightSec,
                missDistanceMeters,
                isHit ? "fast_lead_ok" : "fast_lead_vertical_miss");
    }

    /**
     * Builds an invalid / fallback solution.
     *
     * If actuator state exists, keep the current turret yaw as a safe hold position.
     */
    private static Records.ShotSolution invalidSolution(
            Records.ActuatorState actuatorState,
            double flywheelRpm,
            double hoodAngleDeg,
            String reason) {

        double holdTurretYawRad = actuatorState != null ? actuatorState.turretYawRelRad() : 0.0;

        return new Records.ShotSolution(
                false,
                holdTurretYawRad,
                flywheelRpm,
                hoodAngleDeg,
                0.0,
                0.0,
                999.0,
                reason);
    }

    /**
     * Solves for positive intercept time in the horizontal plane.
     *
     * We want a time t such that:
     *
     * | r / t - v_robot | = shotHorizontalSpeed
     *
     * where:
     * - r = horizontal displacement from muzzle to target
     * - v_robot = robot horizontal velocity
     * - shotHorizontalSpeed = horizontal component of projectile speed
     *
     * This becomes a quadratic in t.
     */
    private static double solveHorizontalInterceptTime(
            double relativeX,
            double relativeY,
            double robotVelX,
            double robotVelY,
            double shotHorizontalSpeed) {

        // Quadratic coefficients: a*t^2 + b*t + c = 0
        double a = (robotVelX * robotVelX + robotVelY * robotVelY)
                - (shotHorizontalSpeed * shotHorizontalSpeed);
        double b = -2.0 * (relativeX * robotVelX + relativeY * robotVelY);
        double c = relativeX * relativeX + relativeY * relativeY;

        // Degenerate case: equation is effectively linear
        if (Math.abs(a) < 1e-9) {
            if (Math.abs(b) < 1e-9) {
                return Double.NaN;
            }

            double t = -c / b;
            return t > 0.0 ? t : Double.NaN;
        }

        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return Double.NaN;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);

        double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
        double t2 = (-b + sqrtDiscriminant) / (2.0 * a);

        // Keep the smallest positive valid solution
        double bestTime = Double.NaN;

        if (t1 > 0.0) {
            bestTime = t1;
        }

        if (t2 > 0.0 && (!Double.isFinite(bestTime) || t2 < bestTime)) {
            bestTime = t2;
        }

        return bestTime;
    }
}