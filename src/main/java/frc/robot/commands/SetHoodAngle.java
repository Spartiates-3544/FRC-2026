package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngle extends Command {
    private final HoodSubsystem hood;
    private final double targetAngleDeg;

    public SetHoodAngle(HoodSubsystem hood, double targetAngleDeg) {
        this.hood = hood;
        this.targetAngleDeg = targetAngleDeg;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setTargetAngleDeg(targetAngleDeg);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Hood/Command Deg", targetAngleDeg);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}