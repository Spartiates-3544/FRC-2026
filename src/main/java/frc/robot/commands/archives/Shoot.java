package frc.robot.commands.archives;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;

    public Shoot(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);
        shooter.setShooterRpm(Constants.Commands.SHOOTER_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setKickerRpm(0.0);
        shooter.setShooterRpm(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}