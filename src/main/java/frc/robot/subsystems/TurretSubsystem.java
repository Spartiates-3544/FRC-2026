package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.robot.Records;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;

public final class TurretSubsystem extends SubsystemBase {
    // =========================
    // Shooter params
    // =========================
    private final Records.ShooterParams shooterParams = Constants.Shooter.PARAMS;

    // =========================
    // Hardware
    // =========================
    private final TalonFX turretMotor = new TalonFX(Constants.Turret.MOTOR_ID, Constants.CAN.rio);
    private final DigitalInput homeSwitch = new DigitalInput(Constants.Turret.HOME_SWITCH_DIO);

    // =========================
    // Requests
    // =========================
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0);

    public TurretSubsystem() {
        applyConfigs();
    }

    // =========================
    // Setup
    // =========================
    private void applyConfigs() {
        turretMotor.getConfigurator().apply(Constants.Turret.CONFIG);
    }

    @Override
    public void periodic() {
    }

    // =========================
    // Position control
    // =========================

    /**
     * Convertit l'angle cible de la tourelle en rotations moteur.
     *
     * Le zéro moteur est placé au limit switch, qui correspond à l'angle max mécanique.
     * Donc en s'éloignant du switch, les rotations moteur deviennent négatives.
     */
    public void setTargetAngleDeg(double targetDeg) {
        double clampedDeg = MathUtils.clamp(
                targetDeg,
                shooterParams.turretMinDeg(),
                shooterParams.turretMaxDeg());

        double turretDegFromHome = clampedDeg - shooterParams.turretMaxDeg();
        double turretRotationsFromHome = turretDegFromHome / Constants.Turret.DEGREES_PER_REVOLUTION;
        double motorRotations = turretRotationsFromHome * Constants.Turret.RATIO;

        turretMotor.setControl(motionMagicRequest.withPosition(motorRotations));
    }

    /**
     * Reconvertit la position moteur actuelle en angle réel de tourelle.
     *
     * Comme le zéro moteur est au home, on repart de l'angle max mécanique.
     */
    public double getAngleDeg() {
        double motorRotations = turretMotor.getPosition().getValueAsDouble();
        double turretRotationsFromHome = motorRotations / Constants.Turret.RATIO;
        double turretDegFromHome = turretRotationsFromHome * Constants.Turret.DEGREES_PER_REVOLUTION;
        double angleDeg = shooterParams.turretMaxDeg() + turretDegFromHome;

        return MathUtils.clamp(
                angleDeg,
                shooterParams.turretMinDeg(),
                shooterParams.turretMaxDeg());
    }

    public double getAngleRad() {
        return Math.toRadians(getAngleDeg());
    }

    public boolean isAtAngleDeg(double targetDeg) {
        return isAtAngleDeg(targetDeg, Constants.Turret.DEFAULT_ANGLE_TOLERANCE_DEG);
    }

    public boolean isAtAngleDeg(double targetDeg, double toleranceDeg) {
        return Math.abs(getAngleDeg() - targetDeg) <= Math.abs(toleranceDeg);
    }

    // =========================
    // Manual control
    // =========================
    public void setManualOutput(double output) {
        turretMotor.set(output);
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    // =========================
    // Homing / sensors
    // =========================
    public boolean isAtHome() {
        return homeSwitch.get();
    }

    public void resetMotorPosition(double positionRotations) {
        turretMotor.setPosition(positionRotations);
    }

    /**
     * Fait avancer la tourelle lentement jusqu'au switch de home,
     * puis remet la position moteur à la valeur de référence.
     */
    public Command home() {
        return Commands.run(() -> turretMotor.set(Constants.Turret.HOMING_OUTPUT), this)
                .until(this::isAtHome)
                .finallyDo(interrupted -> {
                    turretMotor.stopMotor();

                    if (!interrupted) {
                        resetMotorPosition(Constants.Turret.HOME_SENSOR_POSITION_MOTOR_ROT);
                    }
                });
    }

    // =========================
    // Convenience commands
    // =========================
    public Command setTargetAngleCommand(double angleDegrees) {
        return Commands.runOnce(() -> setTargetAngleDeg(angleDegrees), this);
    }
}