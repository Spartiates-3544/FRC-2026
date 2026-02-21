package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tourelle extends SubsystemBase {
    private TalonFX motor = new TalonFX(8);
    private DigitalInput limHoraire = new DigitalInput(0);
    private DigitalInput antiHoraire = new DigitalInput(1);

    public Tourelle(){
        
    }

    
}
