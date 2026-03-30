package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class FieldTargets {
    private FieldTargets() {
    }

    // Given values (inches)
    private static final double FIELD_WIDTH_IN = 317.69;
    private static final double GOAL_FROM_ALLIANCE_WALL_IN = 182.11;
    private static final double GOAL_HEIGHT_IN = 72.00;
    private static final double FIELD_LENGTH_IN = 651.22;
    private static final double GOAL_Y_M = Units.inchesToMeters(FIELD_WIDTH_IN / 2.0);
    private static final double GOAL_Z_M = Units.inchesToMeters(GOAL_HEIGHT_IN);

    // Cached after alliance is first known. Never changes mid-match.
    private static Translation3d cachedGoalCenter = null;

    /**
     * Returns the goal center in field coordinates.
     *
     * The result is computed once after alliance is determined and cached for the
     * rest of the match. This avoids allocating a new Translation3d and calling
     * DriverStation.getAlliance() every 20 ms.
     *
     * Call {@link #clearCache()} from disabledInit() so the cache is refreshed
     * if the robot is re-enabled on the other alliance during testing.
     */
    public static Translation3d goalCenter() {
        if (cachedGoalCenter != null) {
            return cachedGoalCenter;
        }

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            // Alliance not yet reported by the DS — compute but do not cache so
            // we retry next cycle. Default to blue to avoid a null return.
            return computeGoalCenter(false);
        }

        boolean isRed = alliance.get() == DriverStation.Alliance.Red;
        cachedGoalCenter = computeGoalCenter(isRed);
        return cachedGoalCenter;
    }

    private static Translation3d computeGoalCenter(boolean isRed) {
        double xInches = isRed
                ? FIELD_LENGTH_IN - GOAL_FROM_ALLIANCE_WALL_IN
                : GOAL_FROM_ALLIANCE_WALL_IN;
        return new Translation3d(
                Units.inchesToMeters(xInches),
                GOAL_Y_M,
                GOAL_Z_M);
    }

    /**
     * Clears the cached goal position.
     * Call this from Robot.disabledInit() so the cache is refreshed if the robot
     * switches alliances between matches (common during testing).
     */
    public static void clearCache() {
        cachedGoalCenter = null;
    }
}