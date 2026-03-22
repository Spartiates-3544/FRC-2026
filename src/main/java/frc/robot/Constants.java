package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;

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
                true,

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

                2,

                7,
                6,
                7,
                6,
                7,
                6,

                0.8,
                10.0,

                250,
                2500,

                0.5,
                8.0,

                true,
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
            // kP était à 0, probablement pourquoi il bougeais pas
            hoodConfigs.kP = 25.0;
            hoodConfigs.kI = 0.0;
            hoodConfigs.kD = 0.4;
        }

        public static final double hoodRatio = 113.944;
    }

    public static final class Turret {
        private Turret() {
        }

        public static final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

        static {
            turretConfig.Slot0.kP = 12;
            turretConfig.Slot0.kD = 0.4;
            turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }

        public static final double ratio = 7.8125;
    }
}