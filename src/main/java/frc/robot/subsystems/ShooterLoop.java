package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.ExtendedLogger;
import frc.lib.logic.ShooterAim;
import frc.lib.logic.ShooterAim.AimResult;
import frc.lib.robot.Records;
import frc.robot.RobotActStateBuilder;

public class ShooterLoop extends SubsystemBase {
    private final ShooterAim.Loop aimLoop = new ShooterAim.Loop();
    private final RobotActStateBuilder stateBuilder;

    private boolean enabled = false;
    private AimResult lastResult = null;

    @ExtendedLogger.LoggableField(path = "ShooterLoop/LastSolveDurationMs")
    private double lastSolveDurationMs = 0.0;

    public ShooterLoop(
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            TurretSubsystem turret) {

        this.stateBuilder = new RobotActStateBuilder(
                drivetrain::getPose,
                drivetrain::getChassisSpeeds,
                turret::getTourelleAngleRad,
                shooter::getHoodAngleDeg,
                shooter::getShooterRPM);

        ExtendedLogger.registerInstance(this);
    }

    @Override
    public void periodic() {

        if (!enabled) {
            lastResult = null;
            lastSolveDurationMs = 0.0;
            return;
        }

        RobotActStateBuilder.SolverInputs in = stateBuilder.buildAll();

        double t0 = Timer.getFPGATimestamp();

        lastResult = aimLoop.update(
                in.robotState(),
                in.actuatorState(),
                in.target(),
                in.actuatorState().turretYawRelRad());

        double t1 = Timer.getFPGATimestamp();
        lastSolveDurationMs = (t1 - t0) * 1000.0;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        lastResult = null;
        lastSolveDurationMs = 0.0;
        aimLoop.resetWarm();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public AimResult getShotSolution() {
        return lastResult;
    }

    public Records.ShotSolution getCommandedShot() {
        return lastResult != null ? lastResult.cmd() : null;
    }

    public double getLastSolveDurationMs() {
        return lastSolveDurationMs;
    }

    public void resetWarmStart() {
        aimLoop.resetWarm();
    }
}