package frc.robot.subsystems;

import java.lang.Math;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.ExtendedLogger;
import frc.lib.logic.ShooterAim;
import frc.lib.logic.ShooterAim.AimResult;
import frc.lib.robot.Records.ActuatorState;
import frc.robot.Constants;
import frc.robot.FieldTargets;

public class ShooterLoop extends SubsystemBase{
    private ShooterAim.Loop aimLoop = new ShooterAim.Loop();
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private TurretSubsystem turret;

    @ExtendedLogger.LoggableField(path = "ShooterLoop/lastResult")
    private AimResult lastResult = null;
    

    public ShooterLoop(CommandSwerveDrivetrain drive, Shooter shoot, TurretSubsystem tur) {
        drivetrain = drive;
        shooter = shoot;       
         turret = tur;
        ExtendedLogger.registerInstance(this);
    }

    public void update() {
        ActuatorState actState = getActuatorState();
        lastResult = aimLoop.update(drivetrain.getRobotState(), actState, FieldTargets.goalCenter(), actState.turretYawRelRad());
    }

    @Override
    public void periodic() {
        update();
    }

    private ActuatorState getActuatorState() {
        return new ActuatorState(Math.toRadians(turret.getTourelleAngle()), Math.toRadians(Constants.Shooter.defaultParams().hoodFixedDeg()), shooter.getShooterRPM());
    }

    public AimResult getShotSolution() {
        return lastResult;
    }
}
