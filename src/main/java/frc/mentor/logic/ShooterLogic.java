// frc/mentor/logic/ShooterLogic.java
package frc.mentor.logic;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.mentor.robot.Records;
import frc.mentor.utils.MathUtils;

/**
 * <p>
 * Fonctions utilitaires pour le shooter (balistique + géométrie).
 * </p>
 *
 * <p>
 * Ici on met tout ce qui est "calcul pur" et réutilisable:
 * </p>
 * <ul>
 * <li>conversions RPM &lt;-&gt; vitesse de sortie</li>
 * <li>position / direction du muzzle dans le field</li>
 * <li>simulation rapide d'un tir pour mesurer le miss (hit ou non)</li>
 * <li>logique de blind spot du turret (sécurité mécanique et continuité du solveur)</li>
 * </ul>
 *
 * <p>
 * Tout est stateless.
 * </p>
 */
public final class ShooterLogic {
    private ShooterLogic() {
    }

    // -----------------------------
    // Turret blind-spot config
    // -----------------------------
    // !!! À tuner sur le vrai robot !!!
    //
    // Idée:
    // - Il y a des zones où tu ne veux PAS que la turret aille (câbles, hard-stop (limites), bumper, intake, etc.)
    // - On définit une zone interdite par un centre (deg) et une demi-largeur (deg)
    //
    // Exemple: centre 0°, largeur ±12° -> interdit de -12° à +12°.
    private static final double BLIND_CENTER_DEG_1 = 0.0;
    private static final double BLIND_HALF_WIDTH_DEG_1 = 40.0;

    /**
     * <p>
     * Retourne true si le yaw désiré tombe dans une zone interdite (blind spot).
     * </p>
     *
     * <p>
     * Notes:
     * </p>
     * <ul>
     * <li>Le param {@code p} est dans la signature pour garder le pattern uniforme,</li>
     * <li>mais la config blind spot est hardcodée ici (tu peux aussi la mettre dans {@code p} si tu veux).</li>
     * </ul>
     *
     * @param p
     *            paramètres shooter (pas utilisé ici pour l'instant)
     * @param yawRelRad
     *            yaw turret relatif au robot (rad)
     * @return true si le yaw est dans une zone interdite
     */
    public static boolean isTurretBlindSpot(Records.ShooterParams p, double yawRelRad) {
        double deg = MathUtils.wrapDeg(Math.toDegrees(MathUtils.wrapRad(yawRelRad)));
        return MathUtils.inBandDeg(deg, BLIND_CENTER_DEG_1, BLIND_HALF_WIDTH_DEG_1);
    }

    /**
     * <p>
     * Applique un "blind hold": si le yaw désiré est interdit, on garde un yaw safe.
     * </p>
     *
     * <p>
     * Ça permet au solver de continuer à calculer une solution "idéale",
     * pendant que la vrai turret reste en sécurité.
     * Dès que la solution redevient hors blind spot, on reprend tout de suite.
     * </p>
     *
     * @param p
     *            paramètres shooter (limites turret, etc.)
     * @param desiredYawRelRad
     *            yaw désiré (rad)
     * @param holdYawRelRad
     *            yaw à tenir si interdit (souvent: yaw actuel du turret) (rad)
     * @return yaw final à commander (rad), clampé aux limites turret
     */
    public static double applyBlindHoldYaw(Records.ShooterParams p, double desiredYawRelRad, double holdYawRelRad) {
        double desired = clampTurretYaw(p, MathUtils.wrapRad(desiredYawRelRad));
        if (!isTurretBlindSpot(p, desired)) {
            return desired;
        }

        // Hold le dernier yaw safe (typiquement le yaw mesuré du turret)
        return clampTurretYaw(p, MathUtils.wrapRad(holdYawRelRad));
    }

    /**
     * <p>
     * Clamp le yaw du turret dans ses limites mécaniques.
     * </p>
     *
     * @param p
     *            paramètres shooter (turretMinDeg / turretMaxDeg)
     * @param yawRelRad
     *            yaw relatif robot (rad)
     * @return yaw clampé (rad)
     */
    public static double clampTurretYaw(Records.ShooterParams p, double yawRelRad) {
        double lo = Math.toRadians(p.turretMinDeg());
        double hi = Math.toRadians(p.turretMaxDeg());
        return MathUtils.clamp(yawRelRad, lo, hi);
    }

    /**
     * <p>
     * Position du muzzle dans le field (mètres).
     * </p>
     *
     * <p>
     * On part de:
     * </p>
     * <ul>
     * <li>{@code robot.posXY} (field)</li>
     * <li>on ajoute un offset (forward/side) défini dans le repère robot, puis on le rotate par {@code robot.yaw} pour l'amener dans le field</li>
     * <li>{@code z} = hauteur de release ({@code releaseHeight})</li>
     * </ul>
     *
     * @param p
     *            paramètres shooter (offsets + hauteur)
     * @param robot
     *            état robot (pos + yaw)
     * @return position du muzzle dans le repère field (m)
     */
    public static Translation3d muzzlePositionField(Records.ShooterParams p, Records.RobotState robot) {
        Translation2d offRobot = new Translation2d(p.muzzleForwardOffset(), p.muzzleSideOffset());
        Translation2d offField = offRobot.rotateBy(robot.yaw());
        return new Translation3d(
                robot.posXY().getX() + offField.getX(),
                robot.posXY().getY() + offField.getY(),
                p.releaseHeight());
    }

    /**
     * <p>
     * Direction unitaire du muzzle dans le field.
     * </p>
     *
     * <p>
     * Le yaw final du tir est:
     * </p>
     * <ul>
     * <li>{@code yawRobot + yawTurretRel}</li>
     * </ul>
     *
     * <p>
     * L'élévation ({@code hoodDeg}) donne la composante Z.
     * </p>
     *
     * @param robotYawRad
     *            yaw du robot dans le field (rad)
     * @param turretYawRelRad
     *            yaw du turret relatif au robot (rad)
     * @param hoodDeg
     *            angle d'élévation du hood (deg)
     * @return direction (x,y,z) unitaire dans le field
     */
    public static Translation3d muzzleDirectionField(double robotYawRad, double turretYawRelRad, double hoodDeg) {
        double yaw = robotYawRad + turretYawRelRad;
        double elev = Math.toRadians(hoodDeg);
        double ce = Math.cos(elev);
        return new Translation3d(
                Math.cos(yaw) * ce,
                Math.sin(yaw) * ce,
                Math.sin(elev));
    }

    /**
     * <p>
     * Convertit flywheel RPM -&gt; vitesse de sortie de la balle (m/s).
     * </p>
     *
     * <p>
     * On fait:
     * </p>
     * <ul>
     * <li>vitesse périphérique roue = (rpm / 60) * (2*pi*R)</li>
     * <li>on applique {@code slipFactor} (pertes contact)</li>
     * <li>on applique {@code exitSpeedFactor} (calibration globale)</li>
     * </ul>
     *
     * @param p
     *            paramètres shooter (wheelRadius, slipFactor, exitSpeedFactor)
     * @param rpm
     *            vitesse flywheel (RPM)
     * @return vitesse de sortie estimée (m/s)
     */
    public static double rpmToExitSpeed(Records.ShooterParams p, double rpm) {
        double vWheel = (rpm / 60.0) * (2.0 * Math.PI * p.wheelRadiusM());
        return vWheel * p.slipFactor() * p.exitSpeedFactor();
    }

    /**
     * <p>
     * Convertit vitesse de sortie (m/s) -&gt; flywheel RPM.
     * </p>
     *
     * <p>
     * Inverse de {@link #rpmToExitSpeed(Records.ShooterParams, double)}.
     * </p>
     *
     * @param p
     *            paramètres shooter (wheelRadius, slipFactor, exitSpeedFactor)
     * @param v
     *            vitesse de sortie désirée (m/s)
     * @return RPM estimé
     */
    public static double exitSpeedToRpm(Records.ShooterParams p, double v) {
        double denom = (2.0 * Math.PI * p.wheelRadiusM()) * (p.slipFactor() * p.exitSpeedFactor());
        denom = Math.max(denom, 1e-9);
        return (v / denom) * 60.0;
    }

    /**
     * <p>
     * Constante de drag "k" pour une force de drag ~ k * |v| * v.
     * </p>
     *
     * <p>
     * Forme:
     * </p>
     * <ul>
     * <li>Fd = 0.5 * rho * Cd * A * v^2</li>
     * <li>a = F/m -&gt; 0.5*rho*Cd*A*(1/m) * v^2</li>
     * <li>on encode ça comme: a = -k * |v| * v</li>
     * <li>donc k = 0.5*rho*Cd*A*(1/m)</li>
     * </ul>
     *
     * @param p
     *            paramètres physiques (rho, Cd, diamètre, masse)
     * @return k (0 si drag désactivé)
     */
    private static double dragK(Records.ShooterParams p) {
        if (!p.enableDrag()) {
            return 0.0;
        }
        double area = Math.PI * Math.pow(p.ballDiam() * 0.5, 2.0);
        double invM = 1.0 / Math.max(p.ballMass(), 1e-9);
        return 0.5 * p.rhoAir() * p.Cd() * area * invM;
    }

    /**
     * <p>
     * Simulation rapide d'un tir et retourne le miss minimal.
     * </p>
     *
     * <p>
     * Deux modes de goal:
     * </p>
     * <ol>
     * <li>
     * <b>{@code goalType != "top_entry"}</b>
     * <ul>
     * <li>hit si distance 3D &lt;= hitRadius</li>
     * <li>missM = distance 3D minimale (approx)</li>
     * </ul>
     * </li>
     * <li>
     * <b>{@code goalType == "top_entry"}</b>
     * <ul>
     * <li>{@code goalOpenRadiusM} représente le rayon d'ouverture</li>
     * <li>on enlève le rayon de la balle ({@code ballR}) pour éviter les faux hits</li>
     * <li>intersection quand on traverse le plan {@code z=tz}</li>
     * <li>option: {@code goalRequireDescend}: faut que {@code vz &lt; 0} (balle descendante)</li>
     * <li>option: {@code goalRejectSideEntry}: rejette une entrée "par le côté" sous le plan</li>
     * </ul>
     * </li>
     * </ol>
     *
     * @param p
     *            paramètres shooter + simulation (dt, tmax, drag, goal)
     * @param robot
     *            état robot (pose + vitesse)
     * @param turretYawRelRad
     *            yaw turret relatif robot (rad)
     * @param flywheelRpm
     *            rpm flywheel (RPM)
     * @param hoodDeg
     *            hood angle (deg)
     * @param targetXYZ
     *            cible field (m)
     * @return SimResult(hit, t, missM, sideEntry, missXY_m, missZ_m)
     */
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
        double bestMissXY = 0.0;
        double bestMissZ = 0.0;

        boolean hit = false;
        double hitT = 0.0;
        boolean sideEntry = false;

        double hitR2 = p.hitRadiusM() * p.hitRadiusM();
        int steps = (int) Math.floor(tmax / dt);

        double k = dragK(p);

        for (int i = 0; i < steps; i++) {
            double t = i * dt;

            double missXY, missZ;
            double m2;

            if (!useTopEntry) {
                double dx = px - tx, dy = py - ty, dz = pz - tz;
                missXY = Math.hypot(dx, dy);
                missZ = dz;
                m2 = dx * dx + dy * dy + dz * dz;
            } else {
                double rx = px - tx, ry = py - ty;
                double r = Math.hypot(rx, ry);
                double dr = Math.max(0.0, r - rEff);
                double dz = (pz - tz);
                missXY = dr;
                missZ = dz;
                m2 = dr * dr + dz * dz;
            }

            if (m2 < bestMiss2) {
                bestMiss2 = m2;
                bestT = t;
                bestMissXY = missXY;
                bestMissZ = missZ;
            }

            if (pz < 0.0) {
                break;
            }

            if (!useTopEntry) {
                if (m2 <= hitR2) {
                    hit = true;
                    hitT = t;
                    break;
                }
            }

            double px0 = px, py0 = py, pz0 = pz;
            double vz0 = vz;

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

                            bestMiss2 = 0.0;
                            bestMissXY = 0.0;
                            bestMissZ = 0.0;
                            break;
                        }
                    }
                }
            }
        }

        double bestMiss = Math.sqrt(bestMiss2);

        if (sideEntry) {
            bestMiss = Math.max(bestMiss, 9.9);
        }

        return new Records.SimResult(
                hit,
                hit ? hitT : bestT,
                bestMiss,
                sideEntry,
                bestMissXY,
                bestMissZ);
    }
}
