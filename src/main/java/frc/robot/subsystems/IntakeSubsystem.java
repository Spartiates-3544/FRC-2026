package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(Constants.Intake.MOTOR_ID, Constants.CAN.rio);

    private final PneumaticHub pneumaticHub = new PneumaticHub(Constants.Intake.PNEUMATIC_HUB_ID);

    private final Solenoid intakeSolenoid = pneumaticHub.makeSolenoid(Constants.Intake.SOLENOID_CHANNEL);

    public IntakeSubsystem() {
        pneumaticHub.disableCompressor();
        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.Intake.INVERTED;
        config.MotorOutput.NeutralMode = Constants.Intake.NEUTRAL_MODE;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.SPINUP_INTAKE_SMOOTH_TIME_S;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.SUPPLY_CURRENT_LIMIT_A;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.Intake.STATOR_CURRENT_LIMIT_A;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotor.getConfigurator().apply(config);

        setName("Intake");
    }

    public void open() {
        intakeSolenoid.set(true);
    }

    public void close() {
        intakeSolenoid.set(false);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getSpeed() {
        return intakeMotor.get();
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake Speed", this::getSpeed, null);

        // Appeler l'implémentation parent de initSendable()
        super.initSendable(builder);
    }
}