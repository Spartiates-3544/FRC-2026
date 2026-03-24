package frc.robot.subsystems.control;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class Vision {
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

    private final PhotonCamera[] cameras = {
            limelightV2,
            limelightV3,
            heliosLeft,
            heliosRight
    };

    private final PhotonPoseEstimator[] poseEstimators = {
            limelightV2Estimator,
            limelightV3Estimator,
            heliosLeftEstimator,
            heliosRightEstimator
    };

    private final EstimateConsumer consumer;

    public Vision(EstimateConsumer consumer) {
        this.consumer = consumer;
    }

    public void update() {
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator poseEstimator = poseEstimators[i];

            List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
            if (unreadResults.isEmpty()) {
                continue;
            }

            PhotonPipelineResult result = unreadResults.get(unreadResults.size() - 1);

            if (!result.hasTargets()) {
                continue;
            }

            Optional<EstimatedRobotPose> pose = poseEstimator.estimateCoprocMultiTagPose(result);

            if (pose.isEmpty()) {
                pose = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            pose.ifPresent(est -> consumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds));
        }
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp);
    }
}