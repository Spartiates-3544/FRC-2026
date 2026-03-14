package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

public class Spin extends Command {
    private Spindexer spindexer;

    public Spin(Spindexer spinD){
        spindexer = spinD;
        addRequirements(spindexer);

    }
   
    @Override
    public void initialize() {
    }
 
    @Override
    public void execute() {
        spindexer.spin(0.1);
        spindexer.spinAspirateur(-0.4);
      
    }

    @Override
    public void end(boolean interrupted) {
       spindexer.spin(0);
       spindexer.spinAspirateur(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

