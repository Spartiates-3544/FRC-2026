package frc.lib.logic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.robot.Records;
import frc.lib.utils.MathUtils;

/**
 * Fonctions utilitaires pour le shooter (balistique + géométrie).
 *
 * Tout est stateless.
 */
public final class ShooterLogic {
    private ShooterLogic() {
    }

    private static final double EPS = 1e-9;
    private static final double LARGE_MISS2 = 1e30;

    /**
     * Petit container mutable pour éviter de multiplier les tuples locaux.
     */
    private static final class BestState {
        double bestMiss2 = LARGE_MISS2;
        double bestT = 0.0;
        double bestMissXY = 0.0;
        double bestMissZ = 0.0;

        boolean hit = false;
        double hitT = 0.0;
        boolean sideEntry = false;
    }

    public static boolean isTurretBlindSpot(Records.ShooterParams p, double yawRelRad) {
        double halfWidthDeg = p.turretBlindHalfWidthDeg();
        if (halfWidthDeg <= 0.0) {
            return false;
        }

        double deg = MathUtils.wrapDeg(Math.toDegrees(MathUtils.wrapRad(yawRelRad)));
        return MathUtils.inBandDeg(deg, p.turretBlindCenterDeg(), halfWidthDeg);
    }

    public static double applyBlindHoldYaw(Records.ShooterParams p, double desiredYawRelRad, double holdYawRelRad) {
        double desired = clampTurretYaw(p, MathUtils.wrapRad(desiredYawRelRad));
        if (!isTurretBlindSpot(p, desired)) {
            return desired;
        }

        return clampTurretYaw(p, MathUtils.wrapRad(holdYawRelRad));
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

        return new Translation3d(
                Math.cos(yaw) * ce,
                Math.sin(yaw) * ce,
                Math.sin(elev));
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
        if (!p.enableDrag()) {
            return 0.0;
        }

        double area = Math.PI * Math.pow(p.ballDiam() * 0.5, 2.0);
        double invM = 1.0 / Math.max(p.ballMass(), 1e-9);
        return 0.5 * p.rhoAir() * p.Cd() * area * invM;
    }

    /**
     * Simulation rapide d'un tir et retourne le miss minimal.
     *
     * Version optimisée:
     * - split standard / top_entry
     * - split drag / no-drag
     * - moins de sqrt inutiles
     * - segment closest-approach pour le goal standard
     */
    public static Records.SimResult simulateMissFast(
            Records.ShooterParams p,
            Records.RobotState robot,
            double turretYawRelRad,
            double flywheelRpm,
            double hoodDeg,
            Translation3d targetXYZ) {

        final double dt = p.rtDt();
        final double tmax = p.rtTmax();

        turretYawRelRad = clampTurretYaw(p, MathUtils.wrapRad(turretYawRelRad));
        hoodDeg = MathUtils.clamp(hoodDeg, p.hoodMinDeg(), p.hoodMaxDeg());
        flywheelRpm = MathUtils.clamp(flywheelRpm, p.flywheelRpmMin(), p.flywheelRpmMax());

        final double exitSpeed = rpmToExitSpeed(p, flywheelRpm);

        final Translation3d muzzle = muzzlePositionField(p, robot);
        final Translation3d dir = muzzleDirectionField(robot.yaw().getRadians(), turretYawRelRad, hoodDeg);

        final double px0 = muzzle.getX();
        final double py0 = muzzle.getY();
        final double pz0 = muzzle.getZ();

        final double vx0 = robot.velXY().getX() + dir.getX() * exitSpeed;
        final double vy0 = robot.velXY().getY() + dir.getY() * exitSpeed;
        final double vz0 = dir.getZ() * exitSpeed;

        final double tx = targetXYZ.getX();
        final double ty = targetXYZ.getY();
        final double tz = targetXYZ.getZ();

        final boolean useTopEntry = "top_entry".equals(p.goalType());
        final double k = dragK(p);

        if (useTopEntry) {
            if (k > 0.0) {
                return simulateTopEntryWithDrag(p, dt, tmax, k, px0, py0, pz0, vx0, vy0, vz0, tx, ty, tz);
            }
            return simulateTopEntryNoDrag(p, dt, tmax, px0, py0, pz0, vx0, vy0, vz0, tx, ty, tz);
        }

        if (k > 0.0) {
            return simulateStandardWithDrag(p, dt, tmax, k, px0, py0, pz0, vx0, vy0, vz0, tx, ty, tz);
        }
        return simulateStandardNoDrag(p, dt, tmax, px0, py0, pz0, vx0, vy0, vz0, tx, ty, tz);
    }

    private static Records.SimResult simulateStandardNoDrag(
            Records.ShooterParams p,
            double dt,
            double tmax,
            double pxInit,
            double pyInit,
            double pzInit,
            double vxInit,
            double vyInit,
            double vzInit,
            double tx,
            double ty,
            double tz) {

        final int steps = Math.max(1, (int) Math.floor(tmax / dt));
        final double g = p.g();
        final double hitR2 = p.hitRadiusM() * p.hitRadiusM();

        double px = pxInit;
        double py = pyInit;
        double pz = pzInit;

        double vx = vxInit;
        double vy = vyInit;
        double vz = vzInit;

        BestState best = new BestState();

        for (int i = 0; i < steps; i++) {
            final double t = i * dt;

            updateBestStandardPoint(px, py, pz, tx, ty, tz, t, best);

            if (best.bestMiss2 <= hitR2) {
                best.hit = true;
                best.hitT = t;
                break;
            }

            if (pz < 0.0) {
                break;
            }

            final double prevPx = px;
            final double prevPy = py;
            final double prevPz = pz;

            final double prevVz = vz;

            // Exact constant-gravity step for position.
            px = prevPx + vx * dt;
            py = prevPy + vy * dt;
            pz = prevPz + prevVz * dt - 0.5 * g * dt * dt;

            vz = prevVz - g * dt;

            updateBestStandardSegment(prevPx, prevPy, prevPz, px, py, pz, tx, ty, tz, t, dt, best);

            if (best.bestMiss2 <= hitR2) {
                best.hit = true;
                break;
            }

            // Early-out: already past target and getting worse while descending below target.
            if (i > 4) {
                double relx = px - tx;
                double rely = py - ty;
                double awayDot = relx * vx + rely * vy;
                if (awayDot > 0.0 && pz < tz - 0.25 && vz < 0.0 && best.bestMiss2 < 4.0) {
                    break;
                }
            }
        }

        return finalizeStandardResult(best);
    }

    private static Records.SimResult simulateStandardWithDrag(
            Records.ShooterParams p,
            double dt,
            double tmax,
            double k,
            double pxInit,
            double pyInit,
            double pzInit,
            double vxInit,
            double vyInit,
            double vzInit,
            double tx,
            double ty,
            double tz) {

        final int steps = Math.max(1, (int) Math.floor(tmax / dt));
        final double g = p.g();
        final double hitR2 = p.hitRadiusM() * p.hitRadiusM();

        double px = pxInit;
        double py = pyInit;
        double pz = pzInit;

        double vx = vxInit;
        double vy = vyInit;
        double vz = vzInit;

        BestState best = new BestState();

        for (int i = 0; i < steps; i++) {
            final double t = i * dt;

            updateBestStandardPoint(px, py, pz, tx, ty, tz, t, best);

            if (best.bestMiss2 <= hitR2) {
                best.hit = true;
                best.hitT = t;
                break;
            }

            if (pz < 0.0) {
                break;
            }

            final double prevPx = px;
            final double prevPy = py;
            final double prevPz = pz;

            final double speed2 = vx * vx + vy * vy + vz * vz;
            final double speed = Math.sqrt(speed2) + 1e-12;
            final double ks = k * speed;

            final double ax = -ks * vx;
            final double ay = -ks * vy;
            final double az = -g - ks * vz;

            // Semi-implicit / velocity Verlet-ish hybrid.
            final double nextVx = vx + ax * dt;
            final double nextVy = vy + ay * dt;
            final double nextVz = vz + az * dt;

            px += 0.5 * (vx + nextVx) * dt;
            py += 0.5 * (vy + nextVy) * dt;
            pz += 0.5 * (vz + nextVz) * dt;

            vx = nextVx;
            vy = nextVy;
            vz = nextVz;

            updateBestStandardSegment(prevPx, prevPy, prevPz, px, py, pz, tx, ty, tz, t, dt, best);

            if (best.bestMiss2 <= hitR2) {
                best.hit = true;
                break;
            }

            if (i > 4) {
                double relx = px - tx;
                double rely = py - ty;
                double awayDot = relx * vx + rely * vy;
                if (awayDot > 0.0 && pz < tz - 0.25 && vz < 0.0 && best.bestMiss2 < 4.0) {
                    break;
                }
            }
        }

        return finalizeStandardResult(best);
    }

    private static Records.SimResult simulateTopEntryNoDrag(
            Records.ShooterParams p,
            double dt,
            double tmax,
            double pxInit,
            double pyInit,
            double pzInit,
            double vxInit,
            double vyInit,
            double vzInit,
            double tx,
            double ty,
            double tz) {

        final int steps = Math.max(1, (int) Math.floor(tmax / dt));
        final double g = p.g();

        final double ballR = 0.5 * p.ballDiam();
        final double rEff = Math.max(1e-6, p.goalOpenRadiusM() - ballR);

        double px = pxInit;
        double py = pyInit;
        double pz = pzInit;

        double vx = vxInit;
        double vy = vyInit;
        double vz = vzInit;

        BestState best = new BestState();

        for (int i = 0; i < steps; i++) {
            final double t = i * dt;

            updateBestTopEntryPoint(px, py, pz, tx, ty, tz, rEff, t, best);

            if (pz < 0.0) {
                break;
            }

            final double prevPx = px;
            final double prevPy = py;
            final double prevPz = pz;
            final double prevVz = vz;

            px = prevPx + vx * dt;
            py = prevPy + vy * dt;
            pz = prevPz + prevVz * dt - 0.5 * g * dt * dt;

            vz = prevVz - g * dt;

            if (handleTopEntryCrossing(
                    p,
                    prevPx, prevPy, prevPz,
                    px, py, pz,
                    prevVz, vz,
                    tx, ty, tz,
                    rEff,
                    t, dt,
                    best)) {
                break;
            }

            if (i > 4) {
                double relx = px - tx;
                double rely = py - ty;
                double awayDot = relx * vx + rely * vy;
                if (awayDot > 0.0 && pz < tz - 0.15 && vz < 0.0 && best.bestMiss2 < 4.0) {
                    break;
                }
            }
        }

        return finalizeTopEntryResult(best);
    }

    private static Records.SimResult simulateTopEntryWithDrag(
            Records.ShooterParams p,
            double dt,
            double tmax,
            double k,
            double pxInit,
            double pyInit,
            double pzInit,
            double vxInit,
            double vyInit,
            double vzInit,
            double tx,
            double ty,
            double tz) {

        final int steps = Math.max(1, (int) Math.floor(tmax / dt));
        final double g = p.g();

        final double ballR = 0.5 * p.ballDiam();
        final double rEff = Math.max(1e-6, p.goalOpenRadiusM() - ballR);

        double px = pxInit;
        double py = pyInit;
        double pz = pzInit;

        double vx = vxInit;
        double vy = vyInit;
        double vz = vzInit;

        BestState best = new BestState();

        for (int i = 0; i < steps; i++) {
            final double t = i * dt;

            updateBestTopEntryPoint(px, py, pz, tx, ty, tz, rEff, t, best);

            if (pz < 0.0) {
                break;
            }

            final double prevPx = px;
            final double prevPy = py;
            final double prevPz = pz;
            final double prevVz = vz;

            final double speed2 = vx * vx + vy * vy + vz * vz;
            final double speed = Math.sqrt(speed2) + 1e-12;
            final double ks = k * speed;

            final double ax = -ks * vx;
            final double ay = -ks * vy;
            final double az = -g - ks * vz;

            final double nextVx = vx + ax * dt;
            final double nextVy = vy + ay * dt;
            final double nextVz = vz + az * dt;

            px += 0.5 * (vx + nextVx) * dt;
            py += 0.5 * (vy + nextVy) * dt;
            pz += 0.5 * (vz + nextVz) * dt;

            vx = nextVx;
            vy = nextVy;
            vz = nextVz;

            if (handleTopEntryCrossing(
                    p,
                    prevPx, prevPy, prevPz,
                    px, py, pz,
                    prevVz, vz,
                    tx, ty, tz,
                    rEff,
                    t, dt,
                    best)) {
                break;
            }

            if (i > 4) {
                double relx = px - tx;
                double rely = py - ty;
                double awayDot = relx * vx + rely * vy;
                if (awayDot > 0.0 && pz < tz - 0.15 && vz < 0.0 && best.bestMiss2 < 4.0) {
                    break;
                }
            }
        }

        return finalizeTopEntryResult(best);
    }

    private static void updateBestStandardPoint(
            double px,
            double py,
            double pz,
            double tx,
            double ty,
            double tz,
            double t,
            BestState best) {

        double dx = px - tx;
        double dy = py - ty;
        double dz = pz - tz;
        double m2 = dx * dx + dy * dy + dz * dz;

        if (m2 < best.bestMiss2) {
            best.bestMiss2 = m2;
            best.bestT = t;
            best.bestMissXY = Math.sqrt(dx * dx + dy * dy);
            best.bestMissZ = dz;
        }
    }

    private static void updateBestStandardSegment(
            double x0,
            double y0,
            double z0,
            double x1,
            double y1,
            double z1,
            double tx,
            double ty,
            double tz,
            double t0,
            double dt,
            BestState best) {

        double sx = x1 - x0;
        double sy = y1 - y0;
        double sz = z1 - z0;

        double segLen2 = sx * sx + sy * sy + sz * sz;
        if (segLen2 <= EPS) {
            return;
        }

        double qx = tx - x0;
        double qy = ty - y0;
        double qz = tz - z0;

        double alpha = (qx * sx + qy * sy + qz * sz) / segLen2;
        alpha = MathUtils.clamp(alpha, 0.0, 1.0);

        double cx = x0 + alpha * sx;
        double cy = y0 + alpha * sy;
        double cz = z0 + alpha * sz;

        double dx = cx - tx;
        double dy = cy - ty;
        double dz = cz - tz;

        double m2 = dx * dx + dy * dy + dz * dz;
        if (m2 < best.bestMiss2) {
            best.bestMiss2 = m2;
            best.bestT = t0 + alpha * dt;
            best.bestMissXY = Math.sqrt(dx * dx + dy * dy);
            best.bestMissZ = dz;
            if (m2 <= EPS) {
                best.hit = true;
                best.hitT = best.bestT;
            }
        }
    }

    private static void updateBestTopEntryPoint(
            double px,
            double py,
            double pz,
            double tx,
            double ty,
            double tz,
            double rEff,
            double t,
            BestState best) {

        double rx = px - tx;
        double ry = py - ty;
        double r2 = rx * rx + ry * ry;
        double r = Math.sqrt(r2);

        double dr = Math.max(0.0, r - rEff);
        double dz = pz - tz;
        double m2 = dr * dr + dz * dz;

        if (m2 < best.bestMiss2) {
            best.bestMiss2 = m2;
            best.bestT = t;
            best.bestMissXY = dr;
            best.bestMissZ = dz;
        }
    }

    private static boolean handleTopEntryCrossing(
            Records.ShooterParams p,
            double px0,
            double py0,
            double pz0,
            double px1,
            double py1,
            double pz1,
            double vz0,
            double vz1,
            double tx,
            double ty,
            double tz,
            double rEff,
            double t,
            double dt,
            BestState best) {

        double dx0 = px0 - tx;
        double dy0 = py0 - ty;
        double dx1 = px1 - tx;
        double dy1 = py1 - ty;

        double r0sq = dx0 * dx0 + dy0 * dy0;
        double r1sq = dx1 * dx1 + dy1 * dy1;
        double rEff2 = rEff * rEff;

        if (p.goalRejectSideEntry() && (pz0 <= tz) && (pz1 <= tz) && (r0sq > rEff2) && (r1sq <= rEff2)) {
            best.sideEntry = true;
            return true;
        }

        if (pz0 > tz && pz1 <= tz) {
            double denom = (pz0 - tz) - (pz1 - tz);
            if (Math.abs(denom) > EPS) {
                double a01 = (pz0 - tz) / denom;
                a01 = MathUtils.clamp(a01, 0.0, 1.0);

                double xc = px0 + (px1 - px0) * a01;
                double yc = py0 + (py1 - py0) * a01;
                double vzc = vz0 + (vz1 - vz0) * a01;

                double rcx = xc - tx;
                double rcy = yc - ty;
                double rc2 = rcx * rcx + rcy * rcy;

                if (rc2 <= rEff2 && (!p.goalRequireDescend() || vzc < 0.0)) {
                    best.hit = true;
                    best.hitT = t + a01 * dt;
                    best.bestMiss2 = 0.0;
                    best.bestT = best.hitT;
                    best.bestMissXY = 0.0;
                    best.bestMissZ = 0.0;
                    return true;
                }
            }
        }

        return false;
    }

    private static Records.SimResult finalizeStandardResult(BestState best) {
        double bestMiss = Math.sqrt(best.bestMiss2);
        return new Records.SimResult(
                best.hit,
                best.hit ? best.hitT : best.bestT,
                bestMiss,
                false,
                best.bestMissXY,
                best.bestMissZ);
    }

    private static Records.SimResult finalizeTopEntryResult(BestState best) {
        double bestMiss = Math.sqrt(best.bestMiss2);
        if (best.sideEntry) {
            bestMiss = Math.max(bestMiss, 9.9);
        }

        return new Records.SimResult(
                best.hit,
                best.hit ? best.hitT : best.bestT,
                bestMiss,
                best.sideEntry,
                best.bestMissXY,
                best.bestMissZ);
    }
}