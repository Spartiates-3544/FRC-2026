package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ramasseur extends SubsystemBase {
    private TalonFX moteurRamasseur = new TalonFX(1);
    private PneumaticHub pneumaticHub = new PneumaticHub(2);
    private Solenoid solenoidRamasseur = pneumaticHub.makeSolenoid(6);
    
    public void ouvrir() {
        solenoidRamasseur.set(true);
    }

    public void fermer(){
        solenoidRamasseur.set(false);
    }

    public void tourner(double vitesse) {
        moteurRamasseur.set(vitesse);
    }
}