package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    // =========================
    // Hardware
    // =========================
    private final TalonFX indexerMotor = new TalonFX(Constants.Spindexer.INDEXER_MOTOR_ID, Constants.CAN.rio);
    private final TalonFX feedMotor = new TalonFX(Constants.Spindexer.FEED_MOTOR_ID, Constants.CAN.canivore);

    // =========================
    // Control
    // =========================
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }

    public void setFeedSpeed(double speed) {
        feedMotor.set(speed);
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
    }

    public void stopFeed() {
        feedMotor.stopMotor();
    }

    public void stopAll() {
        stopIndexer();
        stopFeed();
    }
}