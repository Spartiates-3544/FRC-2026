package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Unjammer extends SubsystemBase {
    private TalonFX unjammer;

    public Unjammer() {
        unjammer = new TalonFX(10);
    }

    public void set(double speed) {
        unjammer.set(speed);
    }
}
