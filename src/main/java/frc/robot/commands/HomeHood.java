package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;

public class HomeHood extends Command {
    private final HoodSubsystem hood;

    public HomeHood(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setManualOutput(Constants.Hood.HOMING_OUTPUT);
    }

    @Override
    public boolean isFinished() {
        return hood.isHomed();
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
        if (!interrupted) {
            hood.resetMotorPosition(Constants.Hood.HOME_SENSOR_POSITION_MOTOR_ROT);
        }
    }
}
