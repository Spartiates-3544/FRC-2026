package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.robot.Records;

public class ShooterSubsystem extends SubsystemBase{ 

    private TalonFX kickerMotor = new TalonFX(5);
    private TalonFX shooterMotor1 = new TalonFX(6);
    private TalonFX shooterMotor2 = new TalonFX(7);
    private TalonFX hoodMoteur = new TalonFX(9);
// private DigitalInput switchHaut = new DigitalInput(2); seulement une switch en bas 
    private DigitalInput switchBas = new DigitalInput(3);

    public ShooterSubsystem() {
        kickerMotor.getConfigurator().apply(Constants.Shooter.kickerConfigs);
        shooterMotor1.getConfigurator().apply(Constants.Shooter.shooterConfigs);
        shooterMotor2.getConfigurator().apply(Constants.Shooter.shooterConfigs);
        hoodMoteur.getConfigurator().apply(Constants.Shooter.hoodConfigs);

    }

    public void setShooterSpeed(double shooterSpeed) {
       VelocityVoltage demande = new VelocityVoltage(shooterSpeed/60).withSlot(0);
       shooterMotor1.setControl(demande);
       shooterMotor2.setControl(demande);
    }

    public void setKickerSpeed(double kickerSpeed) {
        VelocityVoltage demande = new VelocityVoltage(kickerSpeed/60).withSlot(0);
        kickerMotor.setControl(demande);
    }

    // hoodAngle: degrees 
    public void setHoodAngle(double hoodAngle) { 

        Records.ShooterParams p = Constants.Shooter.defaultParams();
        // TODO: verifier facteur convertion
        if(hoodAngle > p.hoodMinDeg() && hoodAngle < p.hoodMaxDeg()) {
            double positionMoteur = Constants.Shooter.facteurConvertionToursParDegreHood*(hoodAngle-p.hoodMinDeg());
            PositionVoltage demande = new PositionVoltage(positionMoteur)
                                            .withSlot(0)
                                            .withLimitReverseMotion(switchBas.get());
            hoodMoteur.setControl(demande);
        } // todo: add else
        
    }    


    /** Move elevator at given speed (-1.0 to 1.0) */
    public void sethoodMoteur(double speed) {
        hoodMoteur.set(speed);
    }

    /** Stop the elevator motor */
    public void stopHood() {
        hoodMoteur.stopMotor();
    }

    /** Returns true if the limit switch is pressed */
    public boolean isHoodAtHome() {
        return switchBas.get(); 
    }

    /** Reset encoder position to zero */
    public void resetHoodPosition() {
        hoodMoteur.setPosition(0);
    }

    /** Simulated encoder update (replace with real encoder code) */
    public void updatePosition(double delta) {
//        position += delta;
    }

    public double getHoodPosition() {
        return hoodMoteur.getPosition().getValueAsDouble();
    }
}

   
