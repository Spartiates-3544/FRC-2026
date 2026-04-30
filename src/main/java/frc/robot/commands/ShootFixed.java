package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class ShootFixed extends Command {
    private static final double FIXED_RPM = 6000.0;

    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;
    private final ShooterSubsystem shooter;
    private final LedSubsystem leds;

    public ShootFixed(
            IntakeSubsystem intake,
            SpindexerSubsystem spindexer,
            ShooterSubsystem shooter,
            LedSubsystem leds) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.leds = leds;

        addRequirements(intake, spindexer, shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(FIXED_RPM);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);;
        intake.setSpeed(Constants.Commands.INTAKE_SPEED);
        spindexer.setFeedSpeed(Constants.Commands.FEED_SPEED);
        spindexer.setIndexerSpeed(Constants.Commands.SPINDEXER_SPEED);
        leds.requestShootNow();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.stopKicker();
        intake.stop();
        spindexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
