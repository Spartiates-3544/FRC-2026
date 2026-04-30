package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class RunIntakeBack extends Command {
    private final IntakeSubsystem intake;
    private final LedSubsystem leds;

    public RunIntakeBack(IntakeSubsystem intake, LedSubsystem leds) {
        this.intake = intake;
        this.leds = leds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setSpeed(1.0);
        leds.requestIntake();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}