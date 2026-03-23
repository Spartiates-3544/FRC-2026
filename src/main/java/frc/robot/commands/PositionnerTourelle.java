package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class PositionnerTourelle extends Command {
    private final TurretSubsystem turret;

    public PositionnerTourelle(TurretSubsystem turretSubsystem) {
        addRequirements(turretSubsystem);
        turret = turretSubsystem;
    }

    @Override
    public void initialize() {
        turret.setTurretDeg(-60);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}