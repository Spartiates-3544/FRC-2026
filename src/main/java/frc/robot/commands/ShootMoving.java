package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logic.ShooterAim.AimResult;
import frc.lib.robot.Records;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLoop;
import frc.robot.subsystems.TurretSubsystem;

public class ShootMoving extends Command {

    private final Shooter shooter;
    private final TurretSubsystem turret;
    private final ShooterLoop shooterLoop;

    public ShootMoving(Shooter shoot, TurretSubsystem tur, ShooterLoop loop) {
        shooter = shoot;
        turret = tur;
        shooterLoop = loop;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        shooterLoop.enable();
    }

    @Override
    public void execute() {
        AimResult target = shooterLoop.getShotSolution();

        if (target == null || !target.ok()) {
            shooter.setKickerSpeed(0);
            shooter.setShooterSpeed(0);
            return;
        }

        Records.ShotSolution cmd = target.cmd();

        shooter.setShooterSpeed(cmd.flywheelRpm());
        shooter.setHoodAngle(cmd.hoodDeg());
        turret.setTurretDeg(Math.toDegrees(cmd.turretYawRelRad()));
        shooter.setKickerSpeed(-6000);
    }

    @Override
    public void end(boolean interrupted) {
        shooterLoop.disable();

        shooter.setKickerSpeed(0);
        shooter.setShooterSpeed(0);
        shooter.stopHood();
        turret.stopTourelle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}