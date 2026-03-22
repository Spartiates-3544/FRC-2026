package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.ExtendedLogger;
import frc.lib.logging.ExtendedLogger.LoggableField;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;

public final class TurretSubsystem extends SubsystemBase {
    private static final double HOME_SENSOR_POSITION_MOTOR_ROT = 3.30;

    private final TalonFX turret = new TalonFX(8, Constants.CAN.rio);
    private final DigitalInput limAntiHoraire = new DigitalInput(7);

    @LoggableField(path = "Turret/limit")
    private boolean limitStatus = false;

    public TurretSubsystem() {
        ExtendedLogger.registerInstance(this);
        applyConfigs();
    }

    private void applyConfigs() {
        turret.getConfigurator().apply(Constants.Turret.turretConfig);
    }

    @Override
    public void periodic() {
        limitStatus = isTourelleAtHome();
    }

    public void setTurretDeg(double deg) {
        double clampedDeg = MathUtils.clamp(
                deg,
                Constants.Shooter.defaultParams().turretMinDeg(),
                Constants.Shooter.defaultParams().turretMaxDeg());

        double toursTourelle = clampedDeg / 360.0;
        double toursMoteur = toursTourelle * Constants.Turret.ratio;

        PositionVoltage demande = new PositionVoltage(toursMoteur).withSlot(0);
        turret.setControl(demande);
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
        return Commands.run(() -> turret.set(-0.10), this)
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
        return MathUtils.clamp(
                (turret.getPosition().getValueAsDouble() / Constants.Turret.ratio) * 360.0,
                Constants.Shooter.defaultParams().turretMinDeg(),
                Constants.Shooter.defaultParams().turretMaxDeg());
    }

    public double getTourelleAngleRad() {
        return Math.toRadians(getTourelleAngle());
    }
}