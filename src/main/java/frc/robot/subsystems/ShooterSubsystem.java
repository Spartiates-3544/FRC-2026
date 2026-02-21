package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{ 

    private TalonFX kickerMotor = new TalonFX(5);
    private TalonFX shooterMotor1 = new TalonFX(6);
    private TalonFX shooterMotor2 = new TalonFX(7);
    private TalonFX hoodMoteur = new TalonFX(9);
    private DigitalInput switchHaut = new DigitalInput(2);
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

    public void setHoodAngle(double hoodAngle) {
        double positionMoteur = Constants.Shooter.facteurConvertionToursParDegreHood*hoodAngle;
        PositionVoltage demande = new PositionVoltage(positionMoteur)
                                        .withSlot(0)
                                        .withLimitForwardMotion(switchHaut.get())
                                        .withLimitReverseMotion(switchBas.get());
        
        hoodMoteur.setControl(demande);
    }    
}