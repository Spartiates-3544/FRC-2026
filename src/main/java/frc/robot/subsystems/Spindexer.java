package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private TalonFX spinnyBoy;
    private TalonFX aspirateur;

    public Spindexer() {
        spinnyBoy = new TalonFX(3);
        aspirateur = new TalonFX(13, "canivore");
    }

    public void spin(double spinSpeed){
        spinnyBoy.set(spinSpeed);
    }

    public void spinAspirateur(double aspirateurSpin){
        aspirateur.set(aspirateurSpin);
    }   
}
