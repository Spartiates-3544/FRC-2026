package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.robot.Records;
import frc.robot.Constants;
import frc.robot.subsystems.control.ShooterLoop;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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
        Records.ShotSolution shot = shooterLoop.getCommandedShot();

        if (shot == null) {
            shooter.setKickerRpm(0.0);
            return;
        }

        shooter.applyShot(shot);

        double turretTargetDeg = -Math.toDegrees(shot.turretYawRelRad());

        // Inverse la cible de 180° pour correspondre à la convention mécanique actuelle.
        if (turretTargetDeg > 0.0) {
            turretTargetDeg -= 180.0;
        } else {
            turretTargetDeg += 180.0;
        }

        turret.setTargetAngleDeg(turretTargetDeg);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);
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