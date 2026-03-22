package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.robot.Records;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private static final double SHOOTER_RPM_TOLERANCE = 150.0;
    private static final double HOOD_DEG_TOLERANCE = 1.0;

    private final TalonFX kickerMotor = new TalonFX(5, Constants.CAN.rio);
    private final TalonFX shooterMotor1 = new TalonFX(6, Constants.CAN.rio);
    private final TalonFX shooterMotor2 = new TalonFX(7, Constants.CAN.rio);
    private final TalonFX hoodMotor = new TalonFX(9, Constants.CAN.rio);
    private final DigitalInput hoodHomeSwitch = new DigitalInput(8);

    private final VoltageOut sysIdRequest = new VoltageOut(0.0);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> kickerMotor.setControl(sysIdRequest.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Shooter() {
        setName("Shooter");
        configureMotors();
    }

    private void configureMotors() {
        kickerMotor.getConfigurator().apply(Constants.Shooter.kickerConfigs);
        shooterMotor1.getConfigurator().apply(Constants.Shooter.shooterConfigs);
        shooterMotor2.getConfigurator().apply(Constants.Shooter.shooterConfigs);
        hoodMotor.getConfigurator().apply(Constants.Shooter.hoodConfigs);
        shooterMotor2.setControl(new Follower(6, MotorAlignmentValue.Aligned));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setShooterSpeed(double shooterSpeedRPM) {
        shooterMotor1.setControl(
                new VelocityVoltage(rpmToRps(shooterSpeedRPM)).withSlot(0));
    }

    public void setKickerSpeed(double kickerSpeedRPM) {
        kickerMotor.setControl(
                new VelocityVoltage(rpmToRps(kickerSpeedRPM)).withSlot(0));
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public double getShooterRPM() {
        return rpsToRpm(shooterMotor1.getVelocity().getValueAsDouble());
    }

    public void setHoodAngle(double hoodAngleDeg) {
        double clampedDeg = clampHoodAngleDeg(hoodAngleDeg);
        double motorRotations = hoodDegToMotorRotations(clampedDeg);

        PositionVoltage request = new PositionVoltage(motorRotations)
                .withSlot(0)
                .withLimitReverseMotion(isHoodAtHome());

        hoodMotor.setControl(request);
    }

    public void setHoodMotor(double speed) {
        hoodMotor.set(speed);
    }

    public void stopHood() {
        hoodMotor.stopMotor();
    }

    public boolean isHoodAtHome() {
        return hoodHomeSwitch.get();
    }

    public void resetHoodPosition() {
        hoodMotor.setPosition(0.0);
    }

    public double getHoodMotorPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public double getHoodAngleDeg() {
        double motorRot = hoodMotor.getPosition().getValueAsDouble();
        double hoodRot = motorRot / Constants.Shooter.hoodRatio;
        return hoodRot * 360.0;
    }

    public boolean atShooterSpeed(double targetRpm) {
        return Math.abs(getShooterRPM() - targetRpm) <= SHOOTER_RPM_TOLERANCE;
    }

    public boolean atHoodAngle(double targetDeg) {
        return Math.abs(getHoodAngleDeg() - targetDeg) <= HOOD_DEG_TOLERANCE;
    }

    public void setKicker(double speed) {
        kickerMotor.set(speed);
    }

    public void setShooter(double speed) {
        shooterMotor1.set(speed);
    }

    public void stopShooter() {
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    public Command setShooterMotorCommand(double shooterSpeedRPM) {
        return Commands.runOnce(() -> setShooterSpeed(shooterSpeedRPM), this);
    }

    public Command homeHood() {
        return Commands.run(() -> hoodMotor.set(-0.10), this)
                .until(this::isHoodAtHome)
                .finallyDo(interrupted -> {
                    hoodMotor.stopMotor();
                    if (!interrupted) {
                        resetHoodPosition();
                    }
                });
    }

    private static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    private static double rpsToRpm(double rps) {
        return rps * 60.0;
    }

    private static double hoodDegToMotorRotations(double hoodAngleDeg) {
        double hoodRotations = hoodAngleDeg / 360.0;
        return hoodRotations * Constants.Shooter.hoodRatio;
    }

    private static double clampHoodAngleDeg(double hoodAngleDeg) {
        Records.ShooterParams p = Constants.Shooter.defaultParams();
        return Math.max(p.hoodMinDeg(), Math.min(p.hoodMaxDeg(), hoodAngleDeg));
    }
}