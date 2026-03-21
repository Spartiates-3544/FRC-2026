package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class HoodReturnToHome extends Command {
    private final Shooter hood;
    private final double homingSpeed;

    public HoodReturnToHome(Shooter hood, double homingSpeed) {
        this.hood = hood;
        this.homingSpeed = homingSpeed;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        // Start moving toward home
        if (homingSpeed>=0 && homingSpeed <=1.0) {
            hood.setHoodMoteur(-Math.abs(homingSpeed)); // Negative = down
        }
    }

    @Override
    public void execute() {
        // Optional: update simulated encoder
        hood.updatePosition(-0.01);
    }

    @Override
    public boolean isFinished() {
        // Stop when limit switch is triggered
        return hood.isHoodAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        hood.stopHood();
        if (!interrupted) {
            hood.resetHoodPosition(); // Set home position to zero
        }
    }
}


