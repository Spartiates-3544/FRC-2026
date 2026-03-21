// frc/robot/Constants.java
package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;

import frc.lib.robot.Records;
import frc.lib.robot.Records.ShotSolution;

/**
 * <p>
 * Constantes globales du robot.
 * </p>
 *
 * Ici on met:
 * - des valeurs fixes (IDs, offsets, limites)
 * - des paramètres "par défaut" safe
 *
 * Notes:
 * - Si tu veux tuner live, tu gardes ces defaults ici,
 * puis tu lis des overrides via NetworkTables (ShooterTuning).
 */
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
    /**
     * <p>
     * Constantes du shooter (paramètres de modèle + limites).
     * </p> 
     *
     * Ces valeurs sont les "defaults" safe.
     * Tu peux les override live via ShooterTuning sans toucher à ce fichier.
     */
    public static final class Shooter {
        private Shooter() {
        }

        /**
         * <p>
         * Paramètres par défaut du solveur balistique.
         * </p>
         *
         * @return Records.ShooterParams (defaults safe)
         */
        public static Records.ShooterParams defaultParams() {
            return new Records.ShooterParams(
                    9.80665,
                    1.225,
                    0.2267,
                    0.127,

                    0.45,
                    true,

                    0.50,
                    0.00,
                    0.00,

                    0.0762,
                    0.90,
                    0.5,

                    1200,
                    4500,

                    60,
                    50,
                    70,

                    -160,
                    160,

                    0.120,
                    0.4,

                    0.005,
                    3.0,

                    "top_entry",
                    0.55,
                    true,
                    true,

                    840,
                    240,
                    12000,

                    true,
                    true,

                    0.7,

                    0.03,
                    true,
                    1e-6,

                    2,

                    9,
                    10,
                    9,
                    10,
                    9,
                    10,

                    0.8,
                    10.0,

                    250,
                    2500,

                    0.5,
                    8.0,

                    true,
                    2,
                    1.8,
                    1.6,

                    6,
                    0.02);
        }

        public static TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
        static {
            kickerConfigs.Slot0.kP = 0.15948;
            kickerConfigs.Slot0.kI = 0;
            kickerConfigs.Slot0.kD = 0;
            kickerConfigs.Slot0.kS = 0.47087;
            kickerConfigs.Slot0.kV = 0.12977;
            kickerConfigs.Slot0.kA = 0.0018328;
        }
        
        public static TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
        static {
            shooterConfigs.Slot0.kP = 0.18488;
            shooterConfigs.Slot0.kI = 0;
            shooterConfigs.Slot0.kD = 0;
            shooterConfigs.Slot0.kS = 0.16649;
            shooterConfigs.Slot0.kV = 0.12109;
            shooterConfigs.Slot0.kA = 0.017213;
        }
    
        public static Slot0Configs hoodConfigs = new Slot0Configs();
        static{
            hoodConfigs.kP = 0;
            hoodConfigs.kI = 0;
            hoodConfigs.kD = 0;
        }

        // public static double facteurConvertionToursParDegreHood = 0.483;
        public static double hoodRatio = 113.944;
        
    }

    public static final class Turret {
        public static TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        
        static {
            turretConfig.Slot0.kP = 12;
            turretConfig.Slot0.kD = 0.4;
            turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }

        public static double ratio = 7.8125;
    }
}
