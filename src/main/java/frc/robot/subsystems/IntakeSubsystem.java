package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    // =========================
    // Hardware
    // =========================
    private final TalonFX intakeMotor =
            new TalonFX(Constants.Intake.MOTOR_ID, Constants.CAN.rio);
    private final PneumaticHub pneumaticHub =
            new PneumaticHub(Constants.Intake.PNEUMATIC_HUB_ID);
    private final Solenoid intakeSolenoid =
            pneumaticHub.makeSolenoid(Constants.Intake.SOLENOID_CHANNEL);

    // =========================
    // Pneumatics
    // =========================
    public void open() {
        intakeSolenoid.set(true);
    }

    public void close() {
        intakeSolenoid.set(false);
    }

    // =========================
    // Motor control
    // =========================
    public void setSpeed(double speed) {
        intakeMotor.set(Constants.Intake.MOTOR_SIGN * speed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}