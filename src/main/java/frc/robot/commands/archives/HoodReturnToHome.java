// package frc.robot.commands.archives;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterSubsystem;

// public class HoodReturnToHome extends Command {
//     private final ShooterSubsystem shooter;
//     private final double homingOutput;

//     public HoodReturnToHome(ShooterSubsystem shooter, double homingOutput) {
//         this.shooter = shooter;
//         this.homingOutput = homingOutput;
//         addRequirements(shooter);
//     }

//     @Override
//     public void initialize() {
//         if (homingOutput >= 0.0 && homingOutput <= 1.0) {
//             shooter.setHoodManualOutput(-Math.abs(homingOutput));
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return shooter.isHoodAtHome();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooter.stopHood();

//         if (!interrupted) {
//             shooter.resetHoodPosition();
//         }
//     }
// }