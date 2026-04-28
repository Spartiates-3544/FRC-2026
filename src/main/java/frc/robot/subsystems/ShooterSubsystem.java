package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.robot.Records;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    // =========================
    // Hardware
    // =========================
    private final TalonFX kickerMotor = new TalonFX(Constants.Shooter.KICKER_MOTOR_ID, Constants.CAN.rio);
    private final TalonFX shooterMotor1 = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_1_ID, Constants.CAN.rio);
    private final TalonFX shooterMotor2 = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_2_ID, Constants.CAN.rio);

    // =========================
    // Requests
    // =========================
    private final VoltageOut sysIdRequest = new VoltageOut(0.0);
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final VelocityVoltage kickerVelocityRequest = new VelocityVoltage(0.0).withSlot(0);

    // =========================
    // State
    // =========================
    // @ExtendedLogger.LoggableField(path="Shooter/ShooterRPM")
    private volatile double lastCommandedShooterRpm = 0.0;

    private double lastCommandedHoodDeg = 0.0;

    // =========================
    // SysId
    // =========================
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(Constants.Shooter.SYSID_STEP_VOLTS),
                    null,
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> kickerMotor.setControl(sysIdRequest.withOutput(volts.in(Volts))),
                    null,
                    this));

    public ShooterSubsystem() {
        // ExtendedLogger.registerInstance(this);
        setName("Shooter");
        applyConfigs();
    }

    // =========================
    // Setup
    // =========================
    private void applyConfigs() {
        kickerMotor.getConfigurator().apply(Constants.Shooter.kickerConfig);
        shooterMotor1.getConfigurator().apply(Constants.Shooter.shooterConfig);
        shooterMotor2.getConfigurator().apply(Constants.Shooter.shooterConfig);

        shooterMotor2.setControl(
                new Follower(Constants.Shooter.SHOOTER_MOTOR_1_ID, MotorAlignmentValue.Aligned));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Current RPM", getShooterRpm());
        SmartDashboard.putNumber("Shooter/Commanded RPM", lastCommandedShooterRpm);
    }

    // =========================
    // SysId commands
    // =========================
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    // =========================
    // Shooter wheel control
    // =========================
    public void setShooterRpm(double shooterRpm) {
        lastCommandedShooterRpm = shooterRpm;

        shooterMotor1.setControl(
                shooterVelocityRequest.withVelocity(rpmToRps(shooterRpm)));
        shooterMotor2.setControl(
                shooterVelocityRequest.withVelocity(rpmToRps(shooterRpm)));
    }

    public double getShooterRpm() {
        return rpsToRpm(shooterMotor1.getVelocity().getValueAsDouble());
    }

    public boolean isAtShooterRpm(double targetRpm) {
        return Math.abs(getShooterRpm() - targetRpm) <= Constants.Shooter.SHOOTER_RPM_TOLERANCE;
    }

    public void setShooterPercentOutput(double output) {
        shooterMotor1.set(output);
        shooterMotor2.set(output);
    }

    public void stopShooter() {
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    public Command setShooterRpmCommand(double shooterRpm) {
        return Commands.runOnce(() -> setShooterRpm(shooterRpm), this);
    }

    // =========================
    // Kicker control
    // =========================
    public void setKickerRpm(double kickerRpm) {
        kickerMotor.setControl(
                kickerVelocityRequest.withVelocity(rpmToRps(kickerRpm)));
    }

    public void setKickerPercentOutput(double output) {
        kickerMotor.set(output);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    // =========================
    // Shot application
    // =========================
    public void applyShot(Records.ShotSolution shot) {
        if (shot == null) {
            return;
        }

        setShooterRpm(shot.flywheelRpm());
    }

    public boolean isReadyForShot(Records.ShotSolution shot) {
        return shot != null
                && isAtShooterRpm(shot.flywheelRpm());
    }

    // =========================
    // State access
    // =========================
    public double getLastCommandedShooterRpm() {
        return lastCommandedShooterRpm;
    }

    public double getLastCommandedHoodDeg() {
        return lastCommandedHoodDeg;
    }

    // =========================
    // Conversion helpers
    // =========================
    private static double rpmToRps(double rpm) {
        return rpm / Constants.Shooter.SECONDS_PER_MINUTE;
    }

    private static double rpsToRpm(double rps) {
        return rps * Constants.Shooter.SECONDS_PER_MINUTE;
    }
}