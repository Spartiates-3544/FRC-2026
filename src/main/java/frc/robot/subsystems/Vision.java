package frc.robot.subsystems;

import java.io.Console;
import java.util.ArrayList;
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
    private PhotonCamera limelightV2 = new PhotonCamera("limelightV2");
    private PhotonCamera limelightV3 = new PhotonCamera("limelightV3");
    private PhotonCamera heliosLeft = new PhotonCamera("heliosLeft");
    private PhotonCamera heliosRight = new PhotonCamera("heliosRight");

    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private PhotonPoseEstimator limelightV2Estimator = new PhotonPoseEstimator(layout, Constants.Vision.limelightV2Pos);
    private PhotonPoseEstimator limelightV3Estimator = new PhotonPoseEstimator(layout, Constants.Vision.limelightV3Pos);
    private PhotonPoseEstimator heliosLeftEstimator = new PhotonPoseEstimator(layout, Constants.Vision.heliosLeftPos);
    private PhotonPoseEstimator heliosrightEstimator = new PhotonPoseEstimator(layout, Constants.Vision.heliosRightPos);
    private PhotonCamera[] cameras = {limelightV2, limelightV3, heliosLeft, heliosRight};
    private PhotonPoseEstimator[] poseEstimators = {limelightV2Estimator, limelightV3Estimator, heliosLeftEstimator, heliosrightEstimator};

    private EstimateConsumer consumer;
    
    // private PhotonPoseEstimator limelightV3Estimator = new PhotonPoseEstimator(layout, null);
    // private PhotonPoseEstimator heliosLeftEstimator = new PhotonPoseEstimator(layout, null);
    // private PhotonPoseEstimator heliosRightEstimator = new PhotonPoseEstimator(layout, null);
    
    public Vision(EstimateConsumer cons) {
        consumer = cons;
    }

    public void update() {
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator poseEstimator = poseEstimators[i];

            List<PhotonPipelineResult> results = camera.getLatestResult();
            
            for (PhotonPipelineResult result : results) {
                Optional<EstimatedRobotPose> pose = poseEstimator.estimateCoprocMultiTagPose(result);
                if (pose.isEmpty()) {
                    pose = limelightV2Estimator.estimateLowestAmbiguityPose(result);
                }   
    
                pose.ifPresent(
                        est -> {
                            consumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
                        });  
            } 
        }        
    }
    
    // https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp);
    }
}
