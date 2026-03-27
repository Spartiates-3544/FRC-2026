package frc.robot.subsystems.control;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
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
    private Records.RobotState lastRobotState = null;
    private double lastSolveDurationMs = 0.0;

    public ShooterLoop(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            TurretSubsystem turret) {

        stateBuilder = new RobotActStateBuilder(
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
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
        lastRobotState = solverInputs.robotState();
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
        return enabled
                && lastShotSolution != null
                && lastShotSolution.ok();
    }

    public boolean isReadyToShoot() {
        if (!hasValidShot() || lastActuatorState == null) {
            return false;
        }

        double rpmError = Math.abs(lastShotSolution.flywheelRpm() - lastActuatorState.flywheelRpm());

        double yawErrorDeg = Math.abs(Math.toDegrees(
                MathUtils.wrapRad(lastShotSolution.turretYawRelRad() - lastActuatorState.turretYawRelRad())));

        return rpmError <= Constants.Commands.SHOOT_READY_RPM_TOLERANCE
                && yawErrorDeg <= Constants.Commands.SHOOT_READY_YAW_TOLERANCE_DEG;
    }

    public boolean isInZone() {
        if (lastRobotState == null) {
            return false;
        }

        Translation3d target = stateBuilder.buildTarget();

        double dx = target.getX() - lastRobotState.posXY().getX();
        double dy = target.getY() - lastRobotState.posXY().getY();
        double dist = Math.hypot(dx, dy);

        return dist <= Constants.Commands.AUTO_SHOOT_MAX_DISTANCE_M;
    }
}