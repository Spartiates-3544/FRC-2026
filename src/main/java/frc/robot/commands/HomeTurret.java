package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class HomeTurret extends Command {
    private final TurretSubsystem turret;

    public HomeTurret(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
                SmartDashboard.putNumber("Turret/Current AMP", turret.getStatorCurrentA());
        turret.setManualOutput(Constants.Turret.HOMING_OUTPUT);
            }

    @Override
    public boolean isFinished() {
        return turret.isHomed();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        if (!interrupted) {
            turret.resetMotorPosition(Constants.Turret.HOME_SENSOR_POSITION_MOTOR_ROT);
        }
    }
}