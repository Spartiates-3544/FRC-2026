package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.robot.Records;
import frc.robot.Constants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.control.ShooterLoop;

public class ShootMoving extends Command {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final ShooterLoop shooterLoop;
    private final LedSubsystem leds;

    public ShootMoving(ShooterSubsystem shooter, TurretSubsystem turret, ShooterLoop shooterLoop, LedSubsystem leds) {
        this.shooter = shooter;
        this.turret = turret;
        this.shooterLoop = shooterLoop;
        this.leds = leds;
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
            leds.requestShootMoving(false);
            return;
        }

        Records.ShotSolution shot = shooterLoop.getCommandedShot();

        if (shot == null) {
            shooter.setKickerRpm(0.0);
            shooter.stopShooter();
            leds.requestShootMoving(false);
            return;
        }

        shooter.applyShot(shot);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);
        SmartDashboard.putNumber("Shooter/Commanded RPM", shot.flywheelRpm());

        double turretTargetDeg = 180.0 - Math.toDegrees(shot.turretYawRelRad());
        if (turretTargetDeg > 180.0) turretTargetDeg -= 360.0;

        SmartDashboard.putNumber("Shooter/Yaw Deg", turretTargetDeg);
        turret.setTargetAngleDeg(turretTargetDeg);

        leds.requestShootMoving(shooterLoop.isReadyToShoot());
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