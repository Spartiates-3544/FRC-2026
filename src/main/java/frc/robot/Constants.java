package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.logic.FastShooterSolver;
import frc.lib.robot.Records;

public final class Constants {
    private Constants() {
    }

    public static final class Drive {
        public static RobotConfig config;

        static {
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static final class CAN {
        private CAN() {
        }

        public static final CANBus rio = new CANBus("rio");
        public static final CANBus canivore = new CANBus("canivore");
    }

    public static final class Shooter {
        private Shooter() {
        }

        public static final Records.ShooterParams DEFAULT_PARAMS = new Records.ShooterParams(
                9.80665,
                1.225,
                0.2267,
                0.127,

                0.45,
                false,

                0.50,
                0.00,
                0.00,

                0.1016,
                0.90,
                0.3,

                1250,
                5500,

                56.30,
                36.14,
                56.30,

                -150,
                160,
                180.0,
                19.9,

                0.080,
                0.5,

                0.020,
                2.0,
                0.020,

                "top_entry",
                0.55,
                true,
                true,

                1200,
                180,
                6500,

                false,
                true,

                0.7,

                0.03,
                true,
                1e-6,

                1,

                3,
                1,
                3,
                1,
                3,
                1,

                0.8,
                10.0,

                250,
                2500,

                0.5,
                8.0,

                false,
                1,
                1.8,
                1.6,

                4,
                0.02);

        public static Records.ShooterParams defaultParams() {
            return DEFAULT_PARAMS;
        }

        public static final TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
        static {
            kickerConfigs.Slot0.kP = 0.15948;
            kickerConfigs.Slot0.kI = 0;
            kickerConfigs.Slot0.kD = 0;
            kickerConfigs.Slot0.kS = 0.47087;
            kickerConfigs.Slot0.kV = 0.12977;
            kickerConfigs.Slot0.kA = 0.0018328;
        }

        public static final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
        static {
            shooterConfigs.Slot0.kP = 0.18488;
            shooterConfigs.Slot0.kI = 0;
            shooterConfigs.Slot0.kD = 0;
            shooterConfigs.Slot0.kS = 0.16649;
            shooterConfigs.Slot0.kV = 0.12109;
            shooterConfigs.Slot0.kA = 0.017213;
        }

        public static final Slot0Configs hoodConfigs = new Slot0Configs();
        static {
            hoodConfigs.kP = 25.0;
            hoodConfigs.kI = 0.0;
            hoodConfigs.kD = 0.4;
        }

        public static final double hoodRatio = 113.944;
    }

    public static final class Turret {
        private Turret() {
        }

        public static final double ratio = 7.8125;

        /**
         * Motion Magic units are in mechanism sensor rotations and rotations/sec style units.
         * Since we're commanding motor rotations directly in the subsystem, these are motor-side units.
         */
        public static final double motionMagicCruiseVelocityRps = 40.0;
        public static final double motionMagicAccelerationRpsPerSec = 120.0;
        public static final double motionMagicJerkRpsPerSecSq = 600.0;

        public static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

        static {
            turretConfig.Slot0.kP = 12.0;
            turretConfig.Slot0.kI = 0.0;
            turretConfig.Slot0.kD = 0.4;
            turretConfig.Slot0.kS = 0.0;
            turretConfig.Slot0.kV = 0.0;
            turretConfig.Slot0.kA = 0.0;

            turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            MotionMagicConfigs mm = turretConfig.MotionMagic;
            mm.MotionMagicCruiseVelocity = motionMagicCruiseVelocityRps;
            mm.MotionMagicAcceleration = motionMagicAccelerationRpsPerSec;
            mm.MotionMagicJerk = motionMagicJerkRpsPerSecSq;
        }
    }

    private static final double FAST_FIXED_HOOD_DEG = 38.0;

    private static final FastShooterSolver.DistanceRpmTable FAST_RPM_TABLE = new FastShooterSolver.DistanceRpmTable(
            new double[] {
                    1.20, 1.60, 2.00, 2.40, 2.80, 3.20, 3.60, 4.00, 4.40, 4.80
            },
            new double[] {
                    2200, 2350, 2500, 2680, 2875, 3075, 3300, 3550, 3825, 4100
            });

    public static double fastFixedHoodDeg() {
        return FAST_FIXED_HOOD_DEG;
    }

    public static FastShooterSolver.DistanceRpmTable fastRpmTable() {
        return FAST_RPM_TABLE;
    }

    public static final class Vision {
        public static Transform3d limelightV2Pos = new Transform3d(new Translation3d(-0.3, 0.113, 0.282), new Rotation3d(0, 0.349066, 2.79252681));
        public static Transform3d limelightV3Pos = new Transform3d(new Translation3d(-0.3, -0.10671000, 0.291), new Rotation3d(0, 0.349066, -2.79252681));
        public static Transform3d heliosRightPos = new Transform3d(new Translation3d(-0.133747, -0.338695, 0.508336), new Rotation3d(0,0.349066, -0.26179939));
        public static Transform3d heliosLeftPos = new Transform3d(new Translation3d(-0.133747, 0.338695, 0.508336), new Rotation3d(0, 0.349066,0.26179939));
    }
}
