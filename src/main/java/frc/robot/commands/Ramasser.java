package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ramasseur;

public class Ramasser extends Command {
    private Ramasseur ramasseur;

    public Ramasser(Ramasseur ram) {
        ramasseur = ram;
        addRequirements(ramasseur);
    }

    @Override
    public void initialize() {
        ramasseur.ouvrir();
    }
 
    @Override
    public void execute() {
        ramasseur.tourner(1);
    }

    @Override
    public void end(boolean interrupted) {
        ramasseur.fermer();
        ramasseur.tourner(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
