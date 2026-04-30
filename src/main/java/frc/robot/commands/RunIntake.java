package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class RunIntake extends Command {
    private final IntakeSubsystem intake;
    private final LedSubsystem leds;

    public RunIntake(IntakeSubsystem intake, LedSubsystem leds) {
        this.intake = intake;
        this.leds = leds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setSpeed(Constants.Commands.INTAKE_SPEED);
        leds.requestIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}