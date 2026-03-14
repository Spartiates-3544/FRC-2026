package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

import frc.lib.logging.ExtendedLogger;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;

public final class TurretSubsystem extends SubsystemBase {
    private static TalonFX turret = new TalonFX(8);
   /// private DigitalInput limHoraire = new DigitalInput(0);
    private DigitalInput limAntiHoraire = new DigitalInput(7);

    // @Tunables.TunableNum(key = "Shooter/TurretMinDeg", def = -160.0, hz = 2, clamp = false)
    // private double minDeg = 13.0;

    // @Tunables.TunableNum(key = "Shooter/TurretMaxDeg", def = 160.0, hz = 2, clamp = false)
    // private double maxDeg = 13.0;

    // @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    // private double targetDegLog = 0.0;
 
    // @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    // private double posDegLog = 0.0;

    // @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    // private double posMotorRotLog = 0.0;

    // @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    // private double cmdMotorRotLog = 0.0;

    // private double turretTargetDeg = 0.0;

    public TurretSubsystem() {
        ExtendedLogger.registerInstance(this);
        applyConfigs();
    }

    private void applyConfigs() {
        turret.getConfigurator().apply(Constants.Turret.turretConfig);
    }

    @Override
    public void periodic() {
        
    }

    public void setTurretDeg(double deg) {
        double clampedDeg = MathUtils.clamp(deg, -150, 160);
        double toursTourelle = clampedDeg / 360.0;
        double toursMoteur = toursTourelle * Constants.Turret.ratio;

        PositionVoltage demande = new PositionVoltage(toursMoteur)
                .withSlot(0);
               /// .withLimitForwardMotion(limHoraire.get())
                // .withLimitReverseMotion(limAntiHoraire.get());
        
        turret.setControl(demande);
    }

    public void setTourelleMoteur(double speed){
        turret.set(speed);
    }

    public void stopTourelle(){
        turret.stopMotor();
        //turret.get
    }

    public boolean isTourelleAtHome(){
        return limAntiHoraire.get();
    }

    public void resetTourellePosition(double positionRotations){
        turret.setPosition(positionRotations);
    }

    public Command home() {
        return Commands.run(() -> turret.set(0.10), this).until(() -> isTourelleAtHome()).finallyDo(() -> turret.stopMotor()).finallyDo(() -> resetTourellePosition(3.30));
    }

    public Command setTurretPosition(double angleDegrees) {
        return Commands.runOnce(() -> setTurretDeg(angleDegrees), this);
    }

    public double getTourellePosition(){
        return turret.getPosition().getValueAsDouble();
    }

}