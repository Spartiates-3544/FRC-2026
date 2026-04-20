package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.robot.Records;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.control.ShooterLoop;

public class ShootNow extends Command {
    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;
    private final HoodSubsystem hood;
    private final ShooterLoop shooterLoop;
    private final LedSubsystem leds;

    public ShootNow(
            IntakeSubsystem intake,
            SpindexerSubsystem spindexer,
            HoodSubsystem hood,
            ShooterLoop shooterLoop,
            LedSubsystem leds) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.hood = hood;
        this.shooterLoop = shooterLoop;
        this.leds = leds;

        addRequirements(intake, spindexer, hood);
    }

    @Override
    public void initialize() {
        intake.open();
    }

    @Override
    public void execute() {
        Records.ShotSolution shot = shooterLoop.getCommandedShot();
        if (shot != null) {
            hood.setTargetAngleDeg(shot.hoodDeg());
        }

        intake.setSpeed(Constants.Commands.INTAKE_SPEED);
        spindexer.setFeedSpeed(Constants.Commands.FEED_SPEED);
        spindexer.setIndexerSpeed(Constants.Commands.SPINDEXER_SPEED);
        leds.requestShootNow();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intake.close();
        spindexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}