package frc.robot.lib.TagMapper;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;

import java.util.*;

public class TagMapper {
    private static final double MAX_POSE_AMBIGUITY = 0.05; // strict threshold

    private final PhotonCamera camera;
    private Map<Integer, Pose3d> tagMap = new HashMap<>();
    private Pose3d currentPose = new Pose3d();

    private final Map<String, StructPublisher<Pose2d>> publishers = new HashMap<>();

    StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("TagMap/CurrentPose", Pose2d.struct).publish();

    Transform3d cameraTransform3d = new Transform3d(
        new Translation3d(
                Units.inchesToMeters(-12.767),
                Units.inchesToMeters(-12.327),
                Units.inchesToMeters(7.588)),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(135)));


    public TagMapper(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    public void update() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) return;
    
        var target = result.getBestTarget();
        if (target == null || target.getPoseAmbiguity() > MAX_POSE_AMBIGUITY) return;
    
        int tagId = target.getFiducialId();
        Transform3d camToTag = target.getBestCameraToTarget();
    
        // Compute tag pose in field coordinates, relative to the robot:
        // robotPose -> cameraPose -> tagPose
        Pose3d tagPose = currentPose
            .transformBy(cameraTransform3d)
            .transformBy(camToTag);
    
        tagMap.put(tagId, tagPose);
    
        // Update robot pose estimate assuming movement relative to tag
        // tagPose -> inverse(camToTag) -> inverse(cameraTransform3d) = robotPose
        // currentPose = tagPose
        //     .transformBy(camToTag.inverse())
        //     .transformBy(cameraTransform3d.inverse());
        updateCurrentPose();
    
        currentPosePublisher.set(convert3dTo2d(currentPose));
    }

    public Map<Integer, Pose3d> getTagMap() {
        return tagMap;
    }

    public void publishTagMap() {
        for (Map.Entry<Integer, Pose3d> entry : tagMap.entrySet()) {
            int tagId = entry.getKey();
            Pose3d pose3d = entry.getValue();

            // Create publisher if it doesn't exist
            String topicName = "TagMap/Tag_" + tagId;
            
            publishers.computeIfAbsent(topicName, name ->
                NetworkTableInstance.getDefault()
                    .getStructTopic(name, Pose2d.struct)
                    .publish()
            );
            
            // Publish the pose
            publishers.get(topicName).set(convert3dTo2d(pose3d));
        }
    }

    /**
     * update current pose based on robot odometry, cheating! we want to eventually do this based on tag movement.
     * @param initialStartingPose
     */
    public void updateCurrentPose(){
        currentPose = new Pose3d(RobotContainer.drivetrain.getState().Pose);
    }

    private Pose2d convert3dTo2d(Pose3d pose3d){
        // Convert Pose3d to Pose2d (drop Z and use planar rotation)
        Pose2d pose2d = new Pose2d(
            pose3d.getTranslation().getX(),
            pose3d.getTranslation().getY(),
            pose3d.getRotation().toRotation2d()
        );
        return pose2d;
    }
}