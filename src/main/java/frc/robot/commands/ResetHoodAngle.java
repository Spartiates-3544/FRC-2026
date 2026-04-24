package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class ResetHoodAngle extends Command {
    private final HoodSubsystem hood;
    private final double targetAngleDeg;

    public ResetHoodAngle(HoodSubsystem hood, double targetAngleDeg) {
        this.hood = hood;
        this.targetAngleDeg = targetAngleDeg;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.resetMotorPosition(targetAngleDeg);
    }

    @Override
    public void execute() {
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}