// frc/mentor/logic/ShooterTuning.java
package frc.mentor.logic;

import frc.mentor.robot.Config;
import frc.mentor.robot.Records;

/**
 * <p>
 * Tuning live des {@link Records.ShooterParams} via NetworkTables.
 * </p>
 *
 * <p><b>Chemin NT:</b></p>
 * <pre>{@code
 * /Tuning/Shooter/<key>
 * }</pre>
 *
 * <p><b>Usage:</b></p>
 * <pre>{@code
 * Records.ShooterParams defaults = Constants.Shooter.defaultParams();
 *
 * // À chaque loop:
 * Records.ShooterParams pLive = ShooterTuning.params(defaults);
 * }</pre>
 *
 * <p>
 * Note: les defaults sont publiés automatiquement (setDefaultXxx) la première fois que tu lis une clé.
 * </p>
 */
public final class ShooterTuning {
    private ShooterTuning() {
    }

    private static String k(String leaf) {
        return "Shooter/" + leaf;
    }

    /**
     * Retourne une copie de {@link Records.ShooterParams} avec overrides live (si présents dans NT).
     * Le param {@code defaults} est la référence "safe" utilisée pour setDefault + fallback.
     *
     * @param defaults
     *            paramètres par défaut (safe)
     * @return paramètres live (defaults + overrides)
     */
    public static Records.ShooterParams params(Records.ShooterParams defaults) {
        final var d = defaults;

        return new Records.ShooterParams(
                // physiques
                Config.getNumber(k("g"), d.g()),
                Config.getNumber(k("rhoAir"), d.rhoAir()),
                Config.getNumber(k("ballMass"), d.ballMass()),
                Config.getNumber(k("ballDiam"), d.ballDiam()),

                Config.getNumber(k("Cd"), d.Cd()),
                Config.getBoolean(k("enableDrag"), d.enableDrag()),

                // géométrie muzzle
                Config.getNumber(k("releaseHeight"), d.releaseHeight()),
                Config.getNumber(k("muzzleForwardOffset"), d.muzzleForwardOffset()),
                Config.getNumber(k("muzzleSideOffset"), d.muzzleSideOffset()),

                // roues
                Config.getNumber(k("wheelRadiusM"), d.wheelRadiusM()),
                Config.getNumber(k("slipFactor"), d.slipFactor()),
                Config.getNumber(k("exitSpeedFactor"), d.exitSpeedFactor()),

                // rpm limits
                Config.getNumber(k("flywheelRpmMin"), d.flywheelRpmMin()),
                Config.getNumber(k("flywheelRpmMax"), d.flywheelRpmMax()),

                // hood
                Config.getNumber(k("hoodFixedDeg"), d.hoodFixedDeg()),
                Config.getNumber(k("hoodMinDeg"), d.hoodMinDeg()),
                Config.getNumber(k("hoodMaxDeg"), d.hoodMaxDeg()),

                // turret
                Config.getNumber(k("turretMinDeg"), d.turretMinDeg()),
                Config.getNumber(k("turretMaxDeg"), d.turretMaxDeg()),

                // timing
                Config.getNumber(k("fireLatencyS"), d.fireLatencyS()),
                Config.getNumber(k("hitRadiusM"), d.hitRadiusM()),

                // sim dt/tmax
                Config.getNumber(k("rtDt"), d.rtDt()),
                Config.getNumber(k("rtTmax"), d.rtTmax()),

                // goal
                Config.getString(k("goalType"), d.goalType()),
                Config.getNumber(k("goalOpenRadiusM"), d.goalOpenRadiusM()),
                Config.getBoolean(k("goalRequireDescend"), d.goalRequireDescend()),
                Config.getBoolean(k("goalRejectSideEntry"), d.goalRejectSideEntry()),

                // dyn actuators
                Config.getNumber(k("turretSlewRateDps"), d.turretSlewRateDps()),
                Config.getNumber(k("hoodRateDps"), d.hoodRateDps()),
                Config.getNumber(k("flywheelAccelRpmS"), d.flywheelAccelRpmS()),

                // flags solve
                Config.getBoolean(k("solveForHood"), d.solveForHood()),
                Config.getBoolean(k("solveForRpm"), d.solveForRpm()),

                // continuité
                Config.getNumber(k("continuityWeight"), d.continuityWeight()),

                // hood prefer
                Config.getNumber(k("hoodPreferWeight"), d.hoodPreferWeight()),
                Config.getBoolean(k("hoodPreferHigh"), d.hoodPreferHigh()),
                Config.getNumber(k("hoodTiebreakEps"), d.hoodTiebreakEps()),

                // iters
                (int) Config.getNumber(k("instantIters"), d.instantIters()),

                // samples/iters
                (int) Config.getNumber(k("instantYawSamples"), d.instantYawSamples()),
                (int) Config.getNumber(k("instantYawGoldenIters"), d.instantYawGoldenIters()),
                (int) Config.getNumber(k("instantRpmSamples"), d.instantRpmSamples()),
                (int) Config.getNumber(k("instantRpmGoldenIters"), d.instantRpmGoldenIters()),
                (int) Config.getNumber(k("instantHoodSamples"), d.instantHoodSamples()),
                (int) Config.getNumber(k("instantHoodGoldenIters"), d.instantHoodGoldenIters()),

                // windows/spans
                Config.getNumber(k("instantYawWinDegMin"), d.instantYawWinDegMin()),
                Config.getNumber(k("instantYawWinDegMax"), d.instantYawWinDegMax()),

                Config.getNumber(k("instantRpmSpanMin"), d.instantRpmSpanMin()),
                Config.getNumber(k("instantRpmSpanMax"), d.instantRpmSpanMax()),

                Config.getNumber(k("instantHoodSpanMin"), d.instantHoodSpanMin()),
                Config.getNumber(k("instantHoodSpanMax"), d.instantHoodSpanMax()),

                // widen
                Config.getBoolean(k("instantWidenIfMiss"), d.instantWidenIfMiss()),
                (int) Config.getNumber(k("instantWidenPasses"), d.instantWidenPasses()),
                Config.getNumber(k("instantWidenYawMul"), d.instantWidenYawMul()),
                Config.getNumber(k("instantWidenRpmMul"), d.instantWidenRpmMul()),

                // hood pushup
                (int) Config.getNumber(k("hoodPushupSteps"), d.hoodPushupSteps()),
                Config.getNumber(k("hoodPushupEpsM"), d.hoodPushupEpsM()));
    }
}
