package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Unjammer extends SubsystemBase {
    private TalonFX unjammer;

    public Unjammer() {
        unjammer = new TalonFX(10, Constants.CAN.rio);
    }

    public void set(double speed) {
        unjammer.set(speed);
    }
}
