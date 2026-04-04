package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(Constants.Spindexer.INDEXER_MOTOR_ID, Constants.CAN.rio);
    private final TalonFX feedMotor = new TalonFX(Constants.Spindexer.FEED_MOTOR_ID, Constants.CAN.canivore);

    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    // Commanded outputs from the rest of the robot code
    private double commandedIndexerSpeedRPM = 0.0;
    private double commandedFeedSpeed = 0.0;

    // Jam state
    private boolean jamClearingActive = false;
    private double jamClearUntilS = 0.0;
    private double jamAboveThresholdSinceS = -1.0;

    // Useful debug / telemetry values
    private double currentPeakA = 0.0;
    private double currentPeakDecayPerSecond = 25.0;

    public SpindexerSubsystem() {
        indexerMotor.getConfigurator().apply(Constants.Spindexer.indexeurConfig);
        feedMotor.getConfigurator().apply(Constants.Spindexer.feederConfig);
    }

    // public void setIndexerSpeed(double speed) {
    //     commandedIndexerSpeed = clamp(speed);
    // }

    public void setIndexerSpeed(double speedRPM) {
        commandedIndexerSpeedRPM = speedRPM;
    }

    public void setFeedSpeed(double speed) {
        commandedFeedSpeed = clamp(speed);
    }

    public void stopIndexer() {
        commandedIndexerSpeedRPM = 0.0;
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
        commandedIndexerSpeedRPM = 0.0;
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

        boolean shouldBeRunningForward = commandedIndexerSpeedRPM > 10;

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
        indexerMotor.setControl(motionMagicRequest.withVelocity(commandedIndexerSpeedRPM / 60.0));
        feedMotor.set(commandedFeedSpeed);
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}