package frc.robot;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.robot.Records;

public final class RobotActStateBuilder {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final DoubleSupplier turretYawRelRadSupplier;
    private final DoubleSupplier hoodAngleDegSupplier;
    private final DoubleSupplier flywheelRpmSupplier;

    private Translation2d lastFieldVelocity = new Translation2d();
    private double lastVelocityTimestampS = Double.NaN;

    public RobotActStateBuilder(
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            DoubleSupplier turretYawRelRadSupplier,
            DoubleSupplier hoodAngleDegSupplier,
            DoubleSupplier flywheelRpmSupplier) {

        this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier");
        this.chassisSpeedsSupplier = Objects.requireNonNull(chassisSpeedsSupplier, "chassisSpeedsSupplier");
        this.turretYawRelRadSupplier = Objects.requireNonNull(turretYawRelRadSupplier, "turretYawRelRadSupplier");
        this.hoodAngleDegSupplier = Objects.requireNonNull(hoodAngleDegSupplier, "hoodAngleDegSupplier");
        this.flywheelRpmSupplier = Objects.requireNonNull(flywheelRpmSupplier, "flywheelRpmSupplier");
    }

    public Records.RobotState buildRobotState() {
        Pose2d pose = poseSupplier.get();
        ChassisSpeeds speeds = chassisSpeedsSupplier.get();

        Translation2d fieldVelocity = robotToFieldVelocity(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                pose);

        double nowS = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        Translation2d fieldAcceleration;

        if (Double.isNaN(lastVelocityTimestampS)) {
            fieldAcceleration = new Translation2d();
        } else {
            double dt = nowS - lastVelocityTimestampS;
            if (dt > 1e-6) {
                fieldAcceleration = new Translation2d(
                        (fieldVelocity.getX() - lastFieldVelocity.getX()) / dt,
                        (fieldVelocity.getY() - lastFieldVelocity.getY()) / dt);
            } else {
                fieldAcceleration = new Translation2d();
            }
        }

        lastFieldVelocity = fieldVelocity;
        lastVelocityTimestampS = nowS;

        return new Records.RobotState(
                pose.getTranslation(),
                pose.getRotation(),
                fieldVelocity,
                speeds.omegaRadiansPerSecond,
                fieldAcceleration);
    }

    public Records.ActuatorState buildActuatorState() {
        return new Records.ActuatorState(
                turretYawRelRadSupplier.getAsDouble(),
                hoodAngleDegSupplier.getAsDouble(),
                flywheelRpmSupplier.getAsDouble());
    }

    public Translation3d buildTarget() {
        return FieldTargets.goalCenter();
    }

    public SolverInputs buildAll() {
        return new SolverInputs(
                buildRobotState(),
                buildActuatorState(),
                buildTarget());
    }

    private static Translation2d robotToFieldVelocity(double vxRobot, double vyRobot, Pose2d pose) {
        return new Translation2d(vxRobot, vyRobot).rotateBy(pose.getRotation());
    }

    public record SolverInputs(
            Records.RobotState robotState,
            Records.ActuatorState actuatorState,
            Translation3d target) {
    }
}