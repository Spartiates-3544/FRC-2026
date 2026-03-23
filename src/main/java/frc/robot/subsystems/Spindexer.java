package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
    private TalonFX spinnyBoy = new TalonFX(3, Constants.CAN.rio);
    private TalonFX aspirateur = new TalonFX(13, Constants.CAN.canivore);

    public void spin(double spinSpeed){
        spinnyBoy.set(spinSpeed);
    }

    public void spinAspirateur(double aspirateurSpin){
        aspirateur.set(aspirateurSpin);
    }   
}
