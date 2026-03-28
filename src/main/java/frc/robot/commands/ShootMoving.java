package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.robot.Records;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.control.ShooterLoop;

public class ShootMoving extends Command {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final ShooterLoop shooterLoop;

    public ShootMoving(ShooterSubsystem shooter, TurretSubsystem turret, ShooterLoop shooterLoop) {
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
        if (!shooterLoop.isInZone()) {
            shooter.setKickerRpm(0.0);
            shooter.stopShooter();
            return;
        }

        Records.ShotSolution shot = shooterLoop.getCommandedShot();

        if (shot == null || !shot.ok()) {
            shooter.setKickerRpm(0.0);
            shooter.stopShooter();
            return;
        }

        shooter.applyShot(shot);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);

        double turretTargetDeg = Math.toDegrees(shot.turretYawRelRad()) - 180.0;
        if (turretTargetDeg < -180.0) turretTargetDeg += 360.0;
        turret.setTargetAngleDeg(turretTargetDeg);
    }

    @Override
    public void end(boolean interrupted) {
        shooterLoop.disable();
        shooter.setKickerRpm(0.0);
        shooter.stopShooter();
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}