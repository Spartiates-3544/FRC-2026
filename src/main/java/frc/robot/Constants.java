package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.lib.logic.FastShooterSolver;
import frc.lib.robot.Records;

public final class Constants {
    private Constants() {
    }

    public static final class Drive {
        private Drive() {
        }

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

        // =========================
        // Hardware
        // =========================
        public static final int KICKER_MOTOR_ID = 5;
        public static final int SHOOTER_MOTOR_1_ID = 6;
        public static final int SHOOTER_MOTOR_2_ID = 7;
        public static final int HOOD_MOTOR_ID = 9;
        public static final int HOOD_HOME_SWITCH_DIO = 8;

        // =========================
        // Geometry / conversion
        // =========================
        public static final double HOOD_RATIO = 113.944;
        public static final double SECONDS_PER_MINUTE = 60.0;
        public static final double DEGREES_PER_REVOLUTION = 360.0;

        // =========================
        // Behavior / tolerances
        // =========================
        public static final boolean LOOP_ENABLED_BY_DEFAULT = false;
        public static final double SHOOTER_RPM_TOLERANCE = 150.0;
        public static final double HOOD_ANGLE_TOLERANCE_DEG = 1000.0;
        public static final double HOOD_HOME_OUTPUT = -0.10;
        public static final double HOOD_HOME_POSITION_MOTOR_ROT = 0.0;
        public static final double SYSID_STEP_VOLTS = 4.0;

        // =========================
        // Solver params
        // =========================
        public static final Records.ShooterParams PARAMS = new Records.ShooterParams(
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

                -180,
                180,
                0.0,
                30.0,

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

        public static final double FAST_FIXED_HOOD_DEG = 38.0;

        public static final FastShooterSolver.DistanceRpmTable FAST_RPM_TABLE = new FastShooterSolver.DistanceRpmTable(
                new double[] { 1.20, 1.60, 2.00, 2.40, 2.80, 3.20, 3.60, 4.00, 4.40, 4.80, 5.20, 5.60, 6.00, 6.40, 6.80 },
                new double[] { 2300, 2400, 2500, 2600, 2800, 2900, 3000, 3200, 3400, 3600, 3800, 4000, 4100, 4200, 4500 });

        // =========================
        // Configs
        // =========================
        public static final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
        static {
            kickerConfig.Slot0.kP = 0.15948;
            kickerConfig.Slot0.kI = 0;
            kickerConfig.Slot0.kD = 0;
            kickerConfig.Slot0.kS = 0.47087;
            kickerConfig.Slot0.kV = 0.12977;
            kickerConfig.Slot0.kA = 0.0018328;
        }

        public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        static {
            shooterConfig.Slot0.kP = 0.18488;
            shooterConfig.Slot0.kI = 0;
            shooterConfig.Slot0.kD = 0;
            shooterConfig.Slot0.kS = 0.16649;
            shooterConfig.Slot0.kV = 0.12109;
            shooterConfig.Slot0.kA = 0.017213;
        }

        public static final Slot0Configs hoodConfig = new Slot0Configs();
        static {
            hoodConfig.kP = 25.0;
            hoodConfig.kI = 0.0;
            hoodConfig.kD = 0.4;
        }
    }



    public static final class Turret {
        private Turret() {
        }

        public static final int MOTOR_ID = 8;
        public static final int HOME_SWITCH_DIO = 7;

        /** Physical angle limits (deg). 0° = backward, CW positive, +160° = home/limit switch. */
        public static final double ANGLE_MIN_DEG = -150.0;
        public static final double ANGLE_MAX_DEG = 150.0;

        public static final double RATIO = 7.8125;
        public static final double DEGREES_PER_REVOLUTION = 360.0;

        public static final double DEFAULT_ANGLE_TOLERANCE_DEG = 2.0;
        public static final double HOME_SENSOR_POSITION_MOTOR_ROT = 0.0;
        public static final double HOMING_OUTPUT = 0.15;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY_RPS = 40.0;
        public static final double MOTION_MAGIC_ACCELERATION_RPS_PER_SEC = 120.0;
        public static final double MOTION_MAGIC_JERK_RPS_PER_SEC_SQ = 400.0;

        public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();
        static {
            CONFIG.Slot0.kP = 12.0;
            CONFIG.Slot0.kI = 0.0;
            CONFIG.Slot0.kD = 0.4;
            CONFIG.Slot0.kS = 0.0;
            CONFIG.Slot0.kV = 0.0;
            CONFIG.Slot0.kA = 0.0;
            CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            CONFIG.CurrentLimits.SupplyCurrentLimit = 30;
            CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
            CONFIG.CurrentLimits.StatorCurrentLimit = 60;
            CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

            MotionMagicConfigs mm = CONFIG.MotionMagic;
            mm.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY_RPS;
            mm.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION_RPS_PER_SEC;
            mm.MotionMagicJerk = MOTION_MAGIC_JERK_RPS_PER_SEC_SQ;
        }
    }

    public static final class Intake {
        private Intake() {
        }

        public static final int MOTOR_ID = 1;
        public static final int PNEUMATIC_HUB_ID = 2;
        public static final int SOLENOID_CHANNEL = 6;

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final double SPINUP_INTAKE_SMOOTH_TIME_S = 0.50;
        public static final double SUPPLY_CURRENT_LIMIT_A = 25.0;
        public static final double STATOR_CURRENT_LIMIT_A = 40.0;
    }

    public static final class Spindexer {
        private Spindexer() {
        }

        public static final int INDEXER_MOTOR_ID = 3;
        public static final int FEED_MOTOR_ID = 13;

        public static final InvertedValue INDEXER_INVERTED = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue FEED_INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final NeutralModeValue INDEXER_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue FEED_NEUTRAL_MODE = NeutralModeValue.Coast;

        public static final double SPINUP_INDEXER_SMOOTH_TIME_S = 0.50;
        public static final double SPINUP_FEED_SMOOTH_TIME_S = 0.50;

        public static final double INDEXER_SUPPLY_CURRENT_LIMIT_A = 40.0;
        public static final double FEED_SUPPLY_CURRENT_LIMIT_A = 30.0;
        public static final double FEED_STATOR_CURRENT_LIMIT_A = 50.0;

        public static final double JAM_CURRENT_THRESHOLD_A = 55.0;
        public static final double JAM_DEBOUNCE_S = 0.08;
        public static final double JAM_CLEAR_TIME_S = 0.25;
        public static final double JAM_REVERSE_INDEXER_SPEED = -0.25;
        public static final double JAM_REVERSE_FEED_SPEED = -0.35;
    }

    public static final class Unjammer {
        private Unjammer() {
        }

        public static final int MOTOR_ID = 10;

        public static final double SPINUP_SMOOTH_TIME_S = 1.0;
        public static final InvertedValue FEED_INVERTED = InvertedValue.CounterClockwise_Positive;
    }

    public static final class Commands {
        private Commands() {
        }

        public static final double TURRET_PRESET_LEFT_DEG = -90.0;
        public static final double TURRET_PRESET_RIGHT_DEG = 90.0;
        public static final double TURRET_PRESET_DOWN_DEG = 0.0;

        public static final double INTAKE_SPEED = 1.0;
        public static final double SPINDEXER_SPEED = 0.25;
        public static final double FEED_SPEED = -1.0;
        public static final double UNJAMMER_SPEED = 0.5;

        public static final double SHOOTER_RPM = 3544.0;
        public static final double KICKER_RPM = -6000.0;

        public static final double SHOOT_READY_RPM_TOLERANCE = 125.0;
        public static final double SHOOT_READY_YAW_TOLERANCE_DEG = 3.0;
        public static final double AUTO_SHOOT_MAX_DISTANCE_M = 10.0;

        public static final double DUMP_HOOD_DEG = 50.0;
        public static final double DUMP_RPM_AT_WALL = 1600.0;
        public static final double DUMP_RPM_PER_METER = 220.0;
        public static final double DUMP_RPM_MIN = 1600.0;
        public static final double DUMP_RPM_MAX = 4500.0;

        public static final double DUMP_TURRET_TOL_DEG = 5.0;
        public static final double DUMP_HOOD_TOL_DEG = 1.5;
        public static final double DUMP_FLYWHEEL_TOL_RPM = 250.0;

        public static final double SHOOT_NOW_TRANSLATION_SCALE = 0.25;
        public static final double SHOOT_NOW_ROTATION_SCALE = 0.35;
    }

    public static final class Vision {
        private Vision() {
        }

        public static Transform3d LIMELIGHT_V2_POS = new Transform3d(new Translation3d(-0.286, -0.108, 0.304),
                new Rotation3d(0, -Math.toRadians(20), Math.toRadians(160)));
        public static Transform3d LIMELIGHT_V3_POS = new Transform3d(new Translation3d(-0.286, 0.114, 0.304),
                new Rotation3d(0, -Math.toRadians(20),-Math.toRadians(160)));
        public static Transform3d HELIOS_RIGHT_POS = new Transform3d(new Translation3d(-0.133747, -0.338695, 0.508336),
                new Rotation3d(0, -Math.toRadians(20), -Math.toRadians(75)));
        public static Transform3d HELIOS_LEFT_POS = new Transform3d(new Translation3d(-0.133747, 0.338695, 0.508336),
                new Rotation3d(0, -Math.toRadians(20), Math.toRadians(75)));
    }
}