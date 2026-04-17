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
        indexerMotor.getConfigurator().apply(indexerConfig);

        var feedConfig = new TalonFXConfiguration();
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

        if (jamClearingActive) {
            if (nowS < jamClearUntilS) {
                indexerMotor.set(Constants.Spindexer.JAM_REVERSE_INDEXER_SPEED);
                feedMotor.set(commandedFeedSpeed);
                return;
            }

            jamClearingActive = false;
            jamAboveThresholdSinceS = -1.0;
        }

        if (shouldBeRunningForward) {
            if (indexerCurrentA >= Constants.Spindexer.JAM_CURRENT_THRESHOLD_A) {
                if (jamAboveThresholdSinceS < 0.0) {
                    jamAboveThresholdSinceS = nowS;
                } else if ((nowS - jamAboveThresholdSinceS) >= Constants.Spindexer.JAM_DEBOUNCE_S) {
                    jamClearingActive = true;
                    jamClearUntilS = nowS + Constants.Spindexer.JAM_CLEAR_TIME_S;

                    indexerMotor.set(Constants.Spindexer.JAM_REVERSE_INDEXER_SPEED);
                    feedMotor.set(commandedFeedSpeed);
                    return;
                }
            } else {
                jamAboveThresholdSinceS = -1.0;
            }
        } else {
            jamAboveThresholdSinceS = -1.0;
        }

        indexerMotor.set(commandedIndexerSpeed);
        feedMotor.set(commandedFeedSpeed);

    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}