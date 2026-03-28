package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.robot.Records.ShotSolution;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.control.ShooterLoop;

public class SpinUpShooter extends Command {

    private final ShooterSubsystem shooter;
    private final ShooterLoop shooterLoop;
    private double targetRPM = 0.0;

    public SpinUpShooter(ShooterSubsystem shooter, ShooterLoop shooterLoop) {
        this.shooter = shooter;
        this.shooterLoop = shooterLoop;
        addRequirements(shooter);
    }
    @Override
    public void initialize(){
        shooterLoop.enable();
    }

    @Override
    public void execute() {
        ShotSolution beShotSolution = shooterLoop.getCommandedShot();
        targetRPM = beShotSolution.flywheelRpm();
        shooter.applyShot(beShotSolution);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtShooterRpm(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterLoop.disable();
    }
}

