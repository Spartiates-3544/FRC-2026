package frc.robot.subsystems.control;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logic.FastShooterSolver;
import frc.lib.robot.Records;
import frc.robot.Constants;
import frc.robot.RobotActStateBuilder;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterLoop extends SubsystemBase {
    // =========================
    // Dependencies
    // =========================
    private final RobotActStateBuilder stateBuilder;

    // =========================
    // State
    // =========================
    private boolean enabled = Constants.Shooter.LOOP_ENABLED_BY_DEFAULT;
    private Records.ShotSolution lastShotSolution = null;
    private double lastSolveDurationMs = 0.0;

    public ShooterLoop(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            TurretSubsystem turret) {

        stateBuilder = new RobotActStateBuilder(
                drivetrain::getPose,
                drivetrain::getChassisSpeeds,
                () -> -turret.getAngleRad(),
                shooter::getHoodAngleDeg,
                shooter::getShooterRpm);
    }

    @Override
    public void periodic() {
        if (!enabled) {
            clearOutputs();
            return;
        }

        RobotActStateBuilder.SolverInputs solverInputs = stateBuilder.buildAll();

        double startTime = Timer.getFPGATimestamp();

        lastShotSolution = FastShooterSolver.solve(
                Constants.Shooter.PARAMS,
                solverInputs.robotState(),
                solverInputs.actuatorState(),
                solverInputs.target(),
                Constants.Shooter.FAST_RPM_TABLE,
                Constants.Shooter.FAST_FIXED_HOOD_DEG);

        double endTime = Timer.getFPGATimestamp();
        lastSolveDurationMs = (endTime - startTime) * 1000.0;
    }

    private void clearOutputs() {
        lastShotSolution = null;
        lastSolveDurationMs = 0.0;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        clearOutputs();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public Records.ShotSolution getShotSolution() {
        return lastShotSolution;
    }

    public Records.ShotSolution getCommandedShot() {
        return lastShotSolution;
    }

    public double getLastSolveDurationMs() {
        return lastSolveDurationMs;
    }
}