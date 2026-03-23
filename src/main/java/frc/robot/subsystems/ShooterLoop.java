package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logic.FastShooterSolver;
import frc.lib.robot.Records;
import frc.robot.Constants;
import frc.robot.RobotActStateBuilder;

public class ShooterLoop extends SubsystemBase {
    private final RobotActStateBuilder stateBuilder;

    private boolean enabled = false;
    private Records.ShotSolution lastShot = null;
    private double lastSolveDurationMs = 0.0;

    public ShooterLoop(
            CommandSwerveDrivetrain drivetrain,
            Shooter shooter,
            TurretSubsystem turret) {

        this.stateBuilder = new RobotActStateBuilder(
                drivetrain::getPose,
                drivetrain::getChassisSpeeds,
                () -> -turret.getTourelleAngleRad(),
                shooter::getHoodAngleDeg,
                shooter::getShooterRPM);
    }

    @Override
    public void periodic() {
        if (!enabled) {
            lastShot = null;
            lastSolveDurationMs = 0.0;
            return;
        }

        RobotActStateBuilder.SolverInputs in = stateBuilder.buildAll();

        double t0 = Timer.getFPGATimestamp();

        lastShot = FastShooterSolver.solve(
                Constants.Shooter.defaultParams(),
                in.robotState(),
                in.actuatorState(),
                in.target(),
                Constants.fastRpmTable(),
                Constants.fastFixedHoodDeg());

        double t1 = Timer.getFPGATimestamp();
        lastSolveDurationMs = (t1 - t0) * 1000.0;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        lastShot = null;
        lastSolveDurationMs = 0.0;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public Records.ShotSolution getShotSolution() {
        return lastShot;
    }

    public Records.ShotSolution getCommandedShot() {
        return lastShot;
    }

    public double getLastSolveDurationMs() {
        return lastSolveDurationMs;
    }
}