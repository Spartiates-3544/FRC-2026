package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(Constants.Spindexer.INDEXER_MOTOR_ID, Constants.CAN.rio);

    private final TalonFX feedMotor = new TalonFX(Constants.Spindexer.FEED_MOTOR_ID, Constants.CAN.canivore);

    // Commanded outputs from the rest of the robot code
    private double commandedIndexerSpeed = 0.0;
    private double commandedFeedSpeed = 0.0;

    // Jam state
    private boolean jamClearingActive = false;
    private double jamClearUntilS = 0.0;
    private double jamAboveThresholdSinceS = -1.0;

    // Useful debug / telemetry values
    private double currentPeakA = 0.0;
    private double currentPeakDecayPerSecond = 25.0;

    public SpindexerSubsystem() {
        var indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.Inverted = Constants.Spindexer.INDEXER_INVERTED;
        indexerConfig.MotorOutput.NeutralMode = Constants.Spindexer.INDEXER_NEUTRAL_MODE;
        indexerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Spindexer.SPINUP_INDEXER_SMOOTH_TIME_S;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Spindexer.INDEXER_SUPPLY_CURRENT_LIMIT_A;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotor.getConfigurator().apply(indexerConfig);

        var feedConfig = new TalonFXConfiguration();
        feedConfig.MotorOutput.Inverted = Constants.Spindexer.FEED_INVERTED;
        feedConfig.MotorOutput.NeutralMode = Constants.Spindexer.FEED_NEUTRAL_MODE;
        feedConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Spindexer.SPINUP_FEED_SMOOTH_TIME_S;
        feedConfig.CurrentLimits.SupplyCurrentLimit = Constants.Spindexer.FEED_SUPPLY_CURRENT_LIMIT_A;
        feedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feedConfig.CurrentLimits.StatorCurrentLimit = Constants.Spindexer.FEED_STATOR_CURRENT_LIMIT_A;
        feedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feedMotor.getConfigurator().apply(feedConfig);
    }

    public void setIndexerSpeed(double speed) {
        commandedIndexerSpeed = clamp(speed);
    }

    public void setFeedSpeed(double speed) {
        commandedFeedSpeed = clamp(speed);
    }

    public void stopIndexer() {
        commandedIndexerSpeed = 0.0;
        if (!jamClearingActive) {
            indexerMotor.stopMotor();
        }
    }

    public void stopFeed() {
        commandedFeedSpeed = 0.0;
        if (!jamClearingActive) {
            feedMotor.stopMotor();
        }
    }

    public void stopAll() {
        commandedIndexerSpeed = 0.0;
        commandedFeedSpeed = 0.0;
        jamClearingActive = false;
        jamAboveThresholdSinceS = -1.0;
        indexerMotor.stopMotor();
        feedMotor.stopMotor();
    }

    public double getIndexerCurrentA() {
        return indexerMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getCurrentPeakA() {
        return currentPeakA;
    }

    public boolean isJamClearing() {
        return jamClearingActive;
    }

    @Override
    public void periodic() {
        double nowS = Timer.getFPGATimestamp();
        double indexerCurrentA = Math.abs(indexerMotor.getStatorCurrent().getValueAsDouble());

        // Track a decaying "recent peak" for debugging
        currentPeakA = Math.max(indexerCurrentA, currentPeakA - currentPeakDecayPerSecond * 0.02);

        boolean shouldBeRunningForward = commandedIndexerSpeed > 0.05;

        // Si on recule pour débloquer les balles:
        if (jamClearingActive) {
            // Débloquer pendant un certain nombre de secondes
            if (nowS < jamClearUntilS) {
                indexerMotor.set(Constants.Spindexer.JAM_REVERSE_INDEXER_SPEED);
                return;
            }

            // Après, désactiver le mode de débloquage.
            jamClearingActive = false;
            jamAboveThresholdSinceS = -1.0;
        }

        if (shouldBeRunningForward) {
            // Si le moteur de l'indexeur "force":
            if (indexerCurrentA >= Constants.Spindexer.JAM_CURRENT_THRESHOLD_A) {
                // Si jamAboveThresholdSinceS est négatif (ce qui arrive seulement quand on est mode "avancer")
                // Donc si on était en mode "avancer"...
                if (jamAboveThresholdSinceS < 0.0) {
                    // On dit qu'on "jam" depuis le moment présent.
                    jamAboveThresholdSinceS = nowS;
                } else if ((nowS - jamAboveThresholdSinceS) >= Constants.Spindexer.JAM_DEBOUNCE_S) {
                    // Si on jam depuis un certain temps...
                    jamClearingActive = true;
                    jamClearUntilS = nowS + Constants.Spindexer.JAM_CLEAR_TIME_S;

                    indexerMotor.set(Constants.Spindexer.JAM_REVERSE_INDEXER_SPEED);
                    return;
                }
            } else {
                jamAboveThresholdSinceS = -1.0;
            }
        } else {
            jamAboveThresholdSinceS = -1.0;
        }

        // Si on ne doit pas reculer, on met les moteurs à la vitesse désirée.
        indexerMotor.set(commandedIndexerSpeed);
        feedMotor.set(commandedFeedSpeed);
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}