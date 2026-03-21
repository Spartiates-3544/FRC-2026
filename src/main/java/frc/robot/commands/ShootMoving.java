package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logic.ShooterAim.AimResult;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLoop;
import frc.robot.subsystems.TurretSubsystem;

public class ShootMoving extends Command{
    
    private Shooter shooter;
    private TurretSubsystem turret;
    private ShooterLoop shooterLoop;

    public ShootMoving(Shooter shoot, TurretSubsystem tur, ShooterLoop loop) {
        shooter = shoot;
        turret = tur;
        shooterLoop = loop;
        addRequirements(shooter, turret);
    }
 
    @Override
    public void execute() {
        AimResult target = shooterLoop.getShotSolution();

        if (target.ok()) {
            shooter.setShooterSpeed(target.rpmCmd());
            shooter.setKickerSpeed(-6000);
            turret.setTurretDeg(target.yawCmdDeg());
        }
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

