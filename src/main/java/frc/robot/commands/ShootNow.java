package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.UnjammerSubsystem;
import frc.robot.subsystems.control.ShooterLoop;

public class ShootNow extends Command {
    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;
    private final UnjammerSubsystem unjammer;
    private final ShooterLoop shooterLoop;

    public ShootNow(
            IntakeSubsystem intake,
            SpindexerSubsystem spindexer,
            UnjammerSubsystem unjammer,
            ShooterLoop shooterLoop) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.unjammer = unjammer;
        this.shooterLoop = shooterLoop;

        addRequirements(intake, spindexer, unjammer);
    }

    @Override
    public void initialize() {
        intake.open();
    }

    @Override
    public void execute() {
        intake.setSpeed(Constants.Commands.INTAKE_SPEED);
        unjammer.setSpeed(Constants.Commands.UNJAMMER_SPEED);

        if (shooterLoop.isReadyToShoot()) {
            spindexer.setFeedSpeed(Constants.Commands.FEED_SPEED);
            spindexer.setIndexerSpeed(Constants.Commands.SPINDEXER_SPEED);
        } else {
            spindexer.setFeedSpeed(0.0);
            spindexer.setIndexerSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intake.close();
        spindexer.stopAll();
        unjammer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}