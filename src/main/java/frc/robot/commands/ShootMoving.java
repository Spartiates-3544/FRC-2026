package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.robot.Records;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLoop;
import frc.robot.subsystems.TurretSubsystem;

public class ShootMoving extends Command {

    private final Shooter shooter;
    private final TurretSubsystem turret;
    private final ShooterLoop shooterLoop;

    public ShootMoving(Shooter shooter, TurretSubsystem turret, ShooterLoop shooterLoop) {
        this.shooter = shooter;
        this.turret = turret;
        this.shooterLoop = shooterLoop;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        shooterLoop.enable();
    }

    @Override
    public void execute() {
        Records.ShotSolution shot = shooterLoop.getCommandedShot();

        if (shot == null) {
            shooter.setKickerSpeed(0.0);
            return;
        }

        shooter.applyShot(shot);

        double turretTargetDeg = -Math.toDegrees(shot.turretYawRelRad());

        // Reverse turret by 180 degrees
        if (turretTargetDeg > 0.0) {
            turretTargetDeg -= 180.0;
        } else {
            turretTargetDeg += 180.0;
        }

        turret.setTurretDeg(turretTargetDeg);
        shooter.setKickerSpeed(-6000.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooterLoop.disable();

        shooter.setKickerSpeed(0.0);
        shooter.stopShooter();
        turret.stopTourelle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}