package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraPose {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout fieldLayout;

    public PhotonCameraPose() {
        
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            System.out.print("Failed to initialize apriltag layout");
        }

        // Forward Camera
        photonCamera = new PhotonCamera(VisionConstants.kCameraName);

        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
    * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
    *     of the observation. Assumes a planar field and the robot is always firmly on the ground
    */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}