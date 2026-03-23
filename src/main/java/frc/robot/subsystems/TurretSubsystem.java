package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;

public final class TurretSubsystem extends SubsystemBase {
    private static final double TURRET_DEG_TOLERANCE = 2.0;

    /**
     * Mechanical convention:
     * - limit switch side = physical turret MAX angle
     * - after homing, motor position = 0 at the switch
     * - positive commanded angle = toward switch
     * - negative commanded angle = away from switch
     *
     * Observed hardware behavior:
     * - open-loop positive motor output moves toward switch
     * - closed-loop negative motor position moves away from switch
     */
    private static final double HOME_SENSOR_POSITION_MOTOR_ROT = 0.0;

    private final TalonFX turret = new TalonFX(8, Constants.CAN.rio);
    private final DigitalInput limAntiHoraire = new DigitalInput(7);

    private final MotionMagicVoltage turretMotionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0);

    public TurretSubsystem() {
        applyConfigs();
    }

    private void applyConfigs() {
        turret.getConfigurator().apply(Constants.Turret.turretConfig);
    }

    @Override
    public void periodic() {
    }

    public void setTurretDeg(double deg) {
        double clampedDeg = MathUtils.clamp(
                deg,
                Constants.Shooter.defaultParams().turretMinDeg(),
                Constants.Shooter.defaultParams().turretMaxDeg());

        double maxDeg = Constants.Shooter.defaultParams().turretMaxDeg();

        // At home/switch, angle = maxDeg and motor position = 0.
        // Moving away from switch must go NEGATIVE in motor rotations.
        double turretDegFromHome = clampedDeg - maxDeg;
        double turretRotationsFromHome = turretDegFromHome / 360.0;
        double motorRotations = turretRotationsFromHome * Constants.Turret.ratio;

        turret.setControl(turretMotionMagicRequest.withPosition(motorRotations));
    }

    public void setTourelleMoteur(double speed) {
        turret.set(speed);
    }

    public void stopTourelle() {
        turret.stopMotor();
    }

    public boolean isTourelleAtHome() {
        return limAntiHoraire.get();
    }

    public void resetTourellePosition(double positionRotations) {
        turret.setPosition(positionRotations);
    }

    public Command home() {
        return Commands.run(() -> turret.set(0.10), this)
                .until(this::isTourelleAtHome)
                .finallyDo(interrupted -> {
                    turret.stopMotor();
                    if (!interrupted) {
                        resetTourellePosition(HOME_SENSOR_POSITION_MOTOR_ROT);
                    }
                });
    }

    public Command setTurretPosition(double angleDegrees) {
        return Commands.runOnce(() -> setTurretDeg(angleDegrees), this);
    }

    public double getTourelleAngle() {
        double motorRot = turret.getPosition().getValueAsDouble();
        double turretRotFromHome = motorRot / Constants.Turret.ratio;
        double turretDegFromHome = turretRotFromHome * 360.0;

        double maxDeg = Constants.Shooter.defaultParams().turretMaxDeg();
        double angleDeg = maxDeg + turretDegFromHome;

        return MathUtils.clamp(
                angleDeg,
                Constants.Shooter.defaultParams().turretMinDeg(),
                Constants.Shooter.defaultParams().turretMaxDeg());
    }

    public double getTourelleAngleRad() {
        return Math.toRadians(getTourelleAngle());
    }

    public boolean atTurretAngleDeg(double targetDeg) {
        return atTurretAngleDeg(targetDeg, TURRET_DEG_TOLERANCE);
    }

    public boolean atTurretAngleDeg(double targetDeg, double toleranceDeg) {
        return Math.abs(getTourelleAngle() - targetDeg) <= Math.abs(toleranceDeg);
    }
}