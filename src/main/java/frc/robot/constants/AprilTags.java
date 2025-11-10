package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.AllianceSymmetry;

public class AprilTags {
    /**
     * {@link
     * https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json}
     */
    public static final AprilTag[] TAGS = new AprilTag[] {
            // new AprilTag(
            //     1,
            //     new Pose3d(
            //         new Translation3d(16.697198, 0.65532, 1.4859),
            //         new Rotation3d(new Quaternion(0.4539904997395468, 0.0, 0.0,
            // 0.8910065241883678)))),
            // new AprilTag(
            //     2,
            //     new Pose3d(
            //         new Translation3d(16.697198, 7.3964799999999995, 1.4859),
            //         new Rotation3d(
            //             new Quaternion(-0.45399049973954675, -0.0, 0.0, 0.8910065241883679)))),
            // new AprilTag(
            //     3,
            //     new Pose3d(
            //         new Translation3d(11.560809999999998, 8.05561, 1.30175),
            //         new Rotation3d(
            //             new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
            // new AprilTag(
            //     4,
            //     new Pose3d(
            //         new Translation3d(9.276079999999999, 6.137656, 1.8679160000000001),
            //         new Rotation3d(new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074,
            // 0.0)))),
            // new AprilTag(
            //     5,
            //     new Pose3d(
            //         new Translation3d(9.276079999999999, 1.914906, 1.8679160000000001),
            //         new Rotation3d(new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074,
            // 0.0)))),
            new AprilTag(
                    6,
                    new Pose3d(
                            new Translation3d(13.474446, 3.3063179999999996, 0.308102),
                            new Rotation3d(
                                    new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                    7,
                    new Pose3d(
                            new Translation3d(13.890498, 4.0259, 0.308102),
                            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                    8,
                    new Pose3d(
                            new Translation3d(13.474446, 4.745482, 0.308102),
                            new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                    9,
                    new Pose3d(
                            new Translation3d(12.643358, 4.745482, 0.308102),
                            new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(
                    10,
                    new Pose3d(
                            new Translation3d(12.227305999999999, 4.0259, 0.308102),
                            new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
            new AprilTag(
                    11,
                    new Pose3d(
                            new Translation3d(12.643358, 3.3063179999999996, 0.308102),
                            new Rotation3d(
                                    new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
            // new AprilTag(
            //     12,
            //     new Pose3d(
            //         new Translation3d(0.851154, 0.65532, 1.4859),
            //         new Rotation3d(new Quaternion(0.8910065241883679, 0.0, 0.0,
            // 0.45399049973954675)))),
            // new AprilTag(
            //     13,
            //     new Pose3d(
            //         new Translation3d(0.851154, 7.3964799999999995, 1.4859),
            //         new Rotation3d(
            //             new Quaternion(-0.8910065241883678, -0.0, 0.0, 0.45399049973954686)))),
            // new AprilTag(
            //     14,
            //     new Pose3d(
            //         new Translation3d(8.272272, 6.137656, 1.8679160000000001),
            //         new Rotation3d(
            //             new Quaternion(
            //                 5.914589856893349e-17,
            //                 -0.25881904510252074,
            //                 1.5848095757158825e-17,
            //                 0.9659258262890683)))),
            // new AprilTag(
            //     15,
            //     new Pose3d(
            //         new Translation3d(8.272272, 1.914906, 1.8679160000000001),
            //         new Rotation3d(
            //             new Quaternion(
            //                 5.914589856893349e-17,
            //                 -0.25881904510252074,
            //                 1.5848095757158825e-17,
            //                 0.9659258262890683)))),
            // new AprilTag(
            //     16,
            //     new Pose3d(
            //         new Translation3d(5.9875419999999995, -0.0038099999999999996, 1.30175),
            //         new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0,
            // 0.7071067811865476)))),
            new AprilTag(
                    17,
                    new Pose3d(
                            new Translation3d(4.073905999999999, 3.3063179999999996, 0.308102),
                            new Rotation3d(
                                    new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
            new AprilTag(
                    18,
                    new Pose3d(
                            new Translation3d(3.6576, 4.0259, 0.308102),
                            new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
            new AprilTag(
                    19,
                    new Pose3d(
                            new Translation3d(4.073905999999999, 4.745482, 0.308102),
                            new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(
                    20,
                    new Pose3d(
                            new Translation3d(4.904739999999999, 4.745482, 0.308102),
                            new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                    21,
                    new Pose3d(
                            new Translation3d(5.321046, 4.0259, 0.308102),
                            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                    22,
                    new Pose3d(
                            new Translation3d(4.904739999999999, 3.3063179999999996, 0.308102),
                            new Rotation3d(
                                    new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))))
    };

    /**
     * Returns a new array of AprilTags, replacing any tags in TAGS with overrides in TAGS_OVERRIDES if they match by ID.
     * @return AprilTag[] with overrides applied
     */
    public static AprilTag[] getTagsWithOverrides() {
        Map<Integer, AprilTag> overrideMap = new HashMap<>();
        for (AprilTag tag : TAGS_OVERRIDES) {
            overrideMap.put(tag.ID, tag);
        }

        AprilTag[] result = new AprilTag[TAGS.length];
        for (int i = 0; i < TAGS.length; i++) {
            int id = TAGS[i].ID;
            result[i] = overrideMap.getOrDefault(id, TAGS[i]);
        }

        return result;
    }


    public static boolean observableTag(int id) {
        for (AprilTag tag : getTagsWithOverrides()) {
            if (tag.ID == id) {
                if (AllianceSymmetry.isBlue()) {
                    return tag.pose.getX() < FieldConstants.FIELD_LENGTH / 2.0;
                } else {
                    return tag.pose.getX() > FieldConstants.FIELD_LENGTH / 2.0;
                }
            }
        }
        return false;
    }

    // region tag overrides

    public static final AprilTag[] NONE = new AprilTag[] {};

    // used for testing robot odometry, put the tag on the left wall and drive the robot forward.
    public static final AprilTag[] ROBOT_ODOMETRY_TAG = new AprilTag[] {
        new AprilTag(
                22,
                new Pose3d(
                    new Translation3d(2, 8.05561, 0.308102),
                    new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0,
            0.7071067811865476))
            .plus(new Rotation3d(0, 0, Math.PI)) // rotate by 180 deg 
            ))
    };

    // region tag overrides

    public static final AprilTag[] TAGS_OVERRIDES = NONE;
}