package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SpindexerSubsystem;

public class RunIndexer extends Command {
    private final SpindexerSubsystem spindexer;

    public RunIndexer(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void execute() {
        spindexer.setIndexerSpeed(Constants.Commands.SPINDEXER_SPEED);
        spindexer.setFeedSpeed(Constants.Commands.FEED_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}