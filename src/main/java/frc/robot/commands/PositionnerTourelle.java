package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class PositionnerTourelle extends Command {
    private TurretSubsystem turret;

    public PositionnerTourelle(TurretSubsystem turretSubsystem) {
        addRequirements(turretSubsystem);
        turret = turretSubsystem;
    }
    
    @Override
    public void execute() {
        turret.setTurretDeg(-60);
    }


}
