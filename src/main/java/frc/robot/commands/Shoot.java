package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{
    
    private Shooter shooter;

    public Shoot (Shooter shoot1) {
        shooter = shoot1;
        addRequirements(shooter);
    }
 
    @Override
    public void execute() {
        shooter.setKickerSpeed(-1500);
        shooter.setShooterSpeed(1500);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setKickerSpeed(0);
        shooter.setShooterSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

