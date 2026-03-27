package frc.robot.subsystems.control;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public class Vision {
    private static final double FIELD_BORDER_MARGIN_M = 0.35;
    private static final double MAX_ABS_Z_M = 0.75;

    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
    private static final double MAX_SINGLE_TAG_DISTANCE_M = 4.5;
    private static final double MAX_SINGLE_TAG_TRANSLATION_JUMP_M = 1.50;
    private static final double MAX_SINGLE_TAG_ROTATION_JUMP_DEG = 25.0;

    // WPILib guidance: trust gyro heading much more than vision heading for AprilTag poses.
    private static final Matrix<N3, N1> SINGLE_TAG_BASE_STD_DEVS = VecBuilder.fill(0.90, 0.90, 1_000_000.0);
    private static final Matrix<N3, N1> MULTI_TAG_BASE_STD_DEVS = VecBuilder.fill(0.35, 0.35, 1_000_000.0);

    // =========================
    // Cameras
    // =========================
    private final PhotonCamera limelightV2 = new PhotonCamera("limelightV2");
    private final PhotonCamera limelightV3 = new PhotonCamera("limelightV3");
    private final PhotonCamera heliosLeft = new PhotonCamera("heliosLeft");
    private final PhotonCamera heliosRight = new PhotonCamera("heliosRight");

    // =========================
    // Field layout
    // =========================
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // =========================
    // Pose estimators
    // =========================
    private final PhotonPoseEstimator limelightV2Estimator = new PhotonPoseEstimator(layout,
            Constants.Vision.LIMELIGHT_V2_POS);
    private final PhotonPoseEstimator limelightV3Estimator = new PhotonPoseEstimator(layout,
            Constants.Vision.LIMELIGHT_V3_POS);
    private final PhotonPoseEstimator heliosLeftEstimator = new PhotonPoseEstimator(layout,
            Constants.Vision.HELIOS_LEFT_POS);
    private final PhotonPoseEstimator heliosRightEstimator = new PhotonPoseEstimator(layout,
            Constants.Vision.HELIOS_RIGHT_POS);

    private final CameraConfig[] cameras = {
            new CameraConfig("limelightV2", limelightV2, limelightV2Estimator),
            new CameraConfig("limelightV3", limelightV3, limelightV3Estimator),
            new CameraConfig("heliosLeft", heliosLeft, heliosLeftEstimator),
            new CameraConfig("heliosRight", heliosRight, heliosRightEstimator)
    };

    private final EstimateConsumer consumer;
    private final Supplier<Pose2d> currentPoseSupplier;

    public Vision(EstimateConsumer consumer, Supplier<Pose2d> currentPoseSupplier) {
        this.consumer = consumer;
        this.currentPoseSupplier = currentPoseSupplier;
    }

    public void update() {
        Pose2d currentPose = currentPoseSupplier.get();

        for (CameraConfig cam : cameras) {
            List<PhotonPipelineResult> unreadResults = cam.camera.getAllUnreadResults();
            if (unreadResults.isEmpty()) {
                continue;
            }

            for (PhotonPipelineResult result : unreadResults) {
                if (!result.hasTargets()) {
                    continue;
                }

                Optional<EstimatedRobotPose> estimate = cam.estimator.estimateCoprocMultiTagPose(result);

                boolean usedMultiTag = estimate.isPresent() && result.getTargets().size() >= 2;

                if (estimate.isEmpty()) {
                    estimate = cam.estimator.estimateLowestAmbiguityPose(result);
                    usedMultiTag = false;
                }

                if (estimate.isEmpty()) {
                    continue;
                }

                EstimatedRobotPose est = estimate.get();

                if (!isMeasurementUsable(est, result, currentPose, usedMultiTag)) {
                    continue;
                }

                Matrix<N3, N1> stdDevs = getEstimationStdDevs(result, usedMultiTag);

                consumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
            }
        }
    }

    private boolean isMeasurementUsable(
            EstimatedRobotPose est,
            PhotonPipelineResult result,
            Pose2d currentPose,
            boolean usedMultiTag) {

        Pose3d estPose3d = est.estimatedPose;
        Pose2d estPose2d = estPose3d.toPose2d();

        if (!isPoseInsideField(estPose2d)) {
            return false;
        }

        if (Math.abs(estPose3d.getZ()) > MAX_ABS_Z_M) {
            return false;
        }

        if (usedMultiTag) {
            return true;
        }

        // Single-tag fallback: be much harsher.
        PhotonTrackedTarget best = result.getBestTarget();
        if (best == null) {
            return false;
        }

        double ambiguity = best.getPoseAmbiguity();
        if (ambiguity >= 0.0 && ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
            return false;
        }

        double cameraToTagDist = best.getBestCameraToTarget().getTranslation().getNorm();
        if (cameraToTagDist > MAX_SINGLE_TAG_DISTANCE_M) {
            return false;
        }

        double translationJump = estPose2d.getTranslation().getDistance(currentPose.getTranslation());
        if (translationJump > MAX_SINGLE_TAG_TRANSLATION_JUMP_M) {
            return false;
        }

        double rotationJumpDeg = Math.abs(estPose2d.getRotation().minus(currentPose.getRotation()).getDegrees());
        if (rotationJumpDeg > MAX_SINGLE_TAG_ROTATION_JUMP_DEG) {
            return false;
        }

        return true;
    }

    private Matrix<N3, N1> getEstimationStdDevs(
            PhotonPipelineResult result,
            boolean usedMultiTag) {

        int numTargets = result.getTargets().size();
        if (numTargets <= 0) {
            return SINGLE_TAG_BASE_STD_DEVS;
        }

        double avgDist = 0.0;
        for (PhotonTrackedTarget target : result.getTargets()) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTargets;

        Matrix<N3, N1> base = usedMultiTag ? MULTI_TAG_BASE_STD_DEVS : SINGLE_TAG_BASE_STD_DEVS;

        // Distance-based trust scaling, matching WPILib guidance directionally.
        double scale = 1.0 + (avgDist * avgDist / 20.0);

        // Multi-tag gets relatively more trust than single-tag.
        if (usedMultiTag && numTargets > 1) {
            scale /= Math.min(numTargets, 4);
        }

        return base.times(scale);
    }

    private boolean isPoseInsideField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();

        return x >= -FIELD_BORDER_MARGIN_M
                && x <= layout.getFieldLength() + FIELD_BORDER_MARGIN_M
                && y >= -FIELD_BORDER_MARGIN_M
                && y <= layout.getFieldWidth() + FIELD_BORDER_MARGIN_M;
    }

    private record CameraConfig(
            String name,
            PhotonCamera camera,
            PhotonPoseEstimator estimator) {
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }
}