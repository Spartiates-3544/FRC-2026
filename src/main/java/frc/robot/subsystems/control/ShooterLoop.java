package frc.robot.subsystems.control;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logic.FastShooterSolver;
import frc.lib.robot.Records;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;
import frc.robot.RobotActStateBuilder;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterLoop extends SubsystemBase {
    private final RobotActStateBuilder stateBuilder;

    private boolean enabled = Constants.Shooter.LOOP_ENABLED_BY_DEFAULT;
    private Records.ShotSolution lastShotSolution = null;
    private Records.ActuatorState lastActuatorState = null;
    private double lastSolveDurationMs = 0.0;

    public ShooterLoop(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            TurretSubsystem turret) {

        stateBuilder = new RobotActStateBuilder(
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                () -> MathUtils.wrapRad(turret.getAngleRad() + Math.PI),
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
        lastActuatorState = solverInputs.actuatorState();

        if (!isInZone()) {
            clearOutputs();
            return;
        }

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

        SmartDashboard.putNumber("Timing/ShooterLoopSolveMs", lastSolveDurationMs);
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

    public boolean hasValidShot() {
        return enabled && lastShotSolution != null && lastShotSolution.ok();
    }

    public boolean isReadyToShoot() {
        if (!hasValidShot() || lastActuatorState == null) {
            return false;
        }

        double rpmError = Math.abs(lastShotSolution.flywheelRpm() - lastActuatorState.flywheelRpm());
        return rpmError <= Constants.Shooter.SHOOTER_RPM_TOLERANCE;
    }

    public boolean isInZone() {
        return true; // keep your real logic here
    }
}