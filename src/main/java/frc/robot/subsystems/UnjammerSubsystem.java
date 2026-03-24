package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UnjammerSubsystem extends SubsystemBase {
    private final TalonFX motor;

    public UnjammerSubsystem() {
        motor = new TalonFX(Constants.Unjammer.MOTOR_ID, Constants.CAN.rio);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }
}