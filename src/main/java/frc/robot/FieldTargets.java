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

    public static Translation3d goalCenter() {
        boolean isRed = isRedAlliance();

        double xInches;
        if (isRed) {
            xInches = FIELD_LENGTH_IN - GOAL_FROM_ALLIANCE_WALL_IN;
        } else {
            xInches = GOAL_FROM_ALLIANCE_WALL_IN;
        }

        return new Translation3d(
                Units.inchesToMeters(xInches),
                GOAL_Y_M,
                GOAL_Z_M);
    }

    private static boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}