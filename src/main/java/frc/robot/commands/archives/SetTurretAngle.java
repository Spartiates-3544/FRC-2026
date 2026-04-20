package frc.robot.commands.archives;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretAngle extends Command {
    private final TurretSubsystem turret;
    private final double targetAngleDeg;

    public SetTurretAngle(TurretSubsystem turret, double targetAngleDeg) {
        this.turret = turret;
        this.targetAngleDeg = targetAngleDeg;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTargetAngleDeg(targetAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}