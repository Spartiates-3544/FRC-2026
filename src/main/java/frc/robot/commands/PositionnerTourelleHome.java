
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class PositionnerTourelleHome extends Command {
    private TurretSubsystem turret;

    public PositionnerTourelleHome(TurretSubsystem turretSubsystem) {
        addRequirements(turretSubsystem);
        turret = turretSubsystem;
    }
    
    @Override
    public void execute() {
        turret.setTourelleMoteur(-0.2);
    }

    @Override
    public boolean isFinished() {
        return turret.isTourelleAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTourelleMoteur(0);
        if (!interrupted) {
            turret.resetTourellePosition(5.42);
        }
    }
}
