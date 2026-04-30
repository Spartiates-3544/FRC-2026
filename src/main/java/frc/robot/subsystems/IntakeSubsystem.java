package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(Constants.Intake.MOTOR_ID, Constants.CAN.rio);

    public IntakeSubsystem() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.Intake.INVERTED;
        config.MotorOutput.NeutralMode = Constants.Intake.NEUTRAL_MODE;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.SPINUP_INTAKE_SMOOTH_TIME_S;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.SUPPLY_CURRENT_LIMIT_A;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.Intake.STATOR_CURRENT_LIMIT_A;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotor.getConfigurator().apply(config);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}