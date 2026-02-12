// frc/mentor/logic/ShooterLogic.java
package frc.mentor.logic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.mentor.robot.Records;

public final class ShooterLogic {
    private ShooterLogic() {
    }

    public static double clampTurretYaw(Records.ShooterParams p, double yawRelRad) {
        double lo = Math.toRadians(p.turretMinDeg());
        double hi = Math.toRadians(p.turretMaxDeg());
        return MathUtils.clamp(yawRelRad, lo, hi);
    }

    public static Translation3d muzzlePositionField(Records.ShooterParams p, Records.RobotState robot) {
        Translation2d offRobot = new Translation2d(p.muzzleForwardOffset(), p.muzzleSideOffset());
        Translation2d offField = offRobot.rotateBy(robot.yaw());
        return new Translation3d(
                robot.posXY().getX() + offField.getX(),
                robot.posXY().getY() + offField.getY(),
                p.releaseHeight());
    }

    public static Translation3d muzzleDirectionField(double robotYawRad, double turretYawRelRad, double hoodDeg) {
        double yaw = robotYawRad + turretYawRelRad;
        double elev = Math.toRadians(hoodDeg);
        double ce = Math.cos(elev);
        return new Translation3d(Math.cos(yaw) * ce, Math.sin(yaw) * ce, Math.sin(elev));
    }

    public static double rpmToExitSpeed(Records.ShooterParams p, double rpm) {
        double vWheel = (rpm / 60.0) * (2.0 * Math.PI * p.wheelRadiusM());
        return vWheel * p.slipFactor() * p.exitSpeedFactor();
    }

    public static double exitSpeedToRpm(Records.ShooterParams p, double v) {
        double denom = (2.0 * Math.PI * p.wheelRadiusM()) * (p.slipFactor() * p.exitSpeedFactor());
        denom = Math.max(denom, 1e-9);
        return (v / denom) * 60.0;
    }

    private static double dragK(Records.ShooterParams p) {
        // Return 0 if drag disabled (fast path)
        if (!p.enableDrag())
            return 0.0;

        double area = Math.PI * Math.pow(p.ballDiam() * 0.5, 2.0);
        double invM = 1.0 / Math.max(p.ballMass(), 1e-9);
        return 0.5 * p.rhoAir() * p.Cd() * area * invM;
    }

    public static Records.SimResult simulateMissFast(
            Records.ShooterParams p,
            Records.RobotState robot,
            double turretYawRelRad,
            double flywheelRpm,
            double hoodDeg,
            Translation3d targetXYZ) {

        double dt = p.rtDt();
        double tmax = p.rtTmax();

        turretYawRelRad = clampTurretYaw(p, MathUtils.wrapRad(turretYawRelRad));
        hoodDeg = MathUtils.clamp(hoodDeg, p.hoodMinDeg(), p.hoodMaxDeg());
        flywheelRpm = MathUtils.clamp(flywheelRpm, p.flywheelRpmMin(), p.flywheelRpmMax());

        double exitSpeed = rpmToExitSpeed(p, flywheelRpm);

        Translation3d muzzle = muzzlePositionField(p, robot);
        Translation3d dir = muzzleDirectionField(robot.yaw().getRadians(), turretYawRelRad, hoodDeg);

        double px = muzzle.getX(), py = muzzle.getY(), pz = muzzle.getZ();
        double vx = robot.velXY().getX() + dir.getX() * exitSpeed;
        double vy = robot.velXY().getY() + dir.getY() * exitSpeed;
        double vz = dir.getZ() * exitSpeed;

        double tx = targetXYZ.getX(), ty = targetXYZ.getY(), tz = targetXYZ.getZ();

        boolean useTopEntry = "top_entry".equals(p.goalType());
        double ballR = 0.5 * p.ballDiam();
        double rEff = Math.max(1e-6, p.goalOpenRadiusM() - ballR);

        double bestMiss2 = 1e30;
        double bestT = 0.0;
        boolean hit = false;
        double hitT = 0.0;
        boolean sideEntry = false;

        double hitR2 = p.hitRadiusM() * p.hitRadiusM();
        int steps = (int) Math.floor(tmax / dt);
        double k = dragK(p);

        for (int i = 0; i < steps; i++) {
            double t = i * dt;

            double m2;
            if (!useTopEntry) {
                double dx = px - tx, dy = py - ty, dz = pz - tz;
                m2 = dx * dx + dy * dy + dz * dz;
            } else {
                double rx = px - tx, ry = py - ty;
                double r = Math.hypot(rx, ry);
                double dr = Math.max(0.0, r - rEff);
                double dz = (pz - tz);
                m2 = dr * dr + dz * dz;
            }

            if (m2 < bestMiss2) {
                bestMiss2 = m2;
                bestT = t;
            }

            if (pz < 0.0)
                break;

            if (!useTopEntry) {
                if (m2 <= hitR2) {
                    hit = true;
                    hitT = t;
                    break;
                }
            }

            // Keep previous for top-entry crossing logic
            double px0 = px, py0 = py, pz0 = pz;
            double vz0 = vz;

            // ---------
            // accel inline (no allocations)
            // ---------
            double ax = 0.0, ay = 0.0, az = -p.g();
            if (p.enableDrag() && k > 0.0) {
                double speed = Math.sqrt(vx * vx + vy * vy + vz * vz) + 1e-12;
                double ks = k * speed;
                ax -= ks * vx;
                ay -= ks * vy;
                az -= ks * vz;
            }

            vx += ax * dt;
            vy += ay * dt;
            vz += az * dt;

            px += vx * dt;
            py += vy * dt;
            pz += vz * dt;

            if (useTopEntry) {
                double r0 = Math.hypot(px0 - tx, py0 - ty);
                double r1 = Math.hypot(px - tx, py - ty);

                if (p.goalRejectSideEntry() && (pz0 <= tz) && (pz <= tz) && (r0 > rEff) && (r1 <= rEff)) {
                    sideEntry = true;
                    break;
                }

                if (pz0 > tz && pz <= tz) {
                    double denom = (pz0 - tz) - (pz - tz);
                    if (Math.abs(denom) > 1e-9) {
                        double a01 = (pz0 - tz) / denom;
                        a01 = MathUtils.clamp(a01, 0.0, 1.0);

                        double xc = px0 + (px - px0) * a01;
                        double yc = py0 + (py - py0) * a01;
                        double vzc = vz0 + (vz - vz0) * a01;

                        double rc = Math.hypot(xc - tx, yc - ty);
                        if (rc <= rEff && (!p.goalRequireDescend() || vzc < 0.0)) {
                            hit = true;
                            hitT = t + a01 * dt;
                            break;
                        }
                    }
                }
            }
        }

        double bestMiss = Math.sqrt(bestMiss2);
        if (sideEntry)
            bestMiss = Math.max(bestMiss, 9.9);

        return new Records.SimResult(hit, hit ? hitT : bestT, bestMiss, sideEntry);
    }
}
