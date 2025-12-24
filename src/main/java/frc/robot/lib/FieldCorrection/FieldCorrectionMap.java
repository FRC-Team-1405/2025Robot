package frc.robot.lib.FieldCorrection;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.ReefSelecter.Coral;

/**
 * See {@link AprilTagLayout.png the aprilTagLayout}.
 */
public class FieldCorrectionMap {
    // a record is a simple generated object that automatically provides hash and equals methods for the given generics
    public record AllianceCoralKey (Alliance alliance, Coral coral) {}

    // List of Field Correction Maps

    // Positive X moves away from face (forward from face's perspective), Positive Y moves to the right of a face (left from face's perspective)
    // Y: pos = left / neg = right, tag's perspective
    private static Map<AllianceCoralKey, Transform2d> home = Map.of (
        new AllianceCoralKey(Alliance.Blue, Coral.Position_5),   new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.75)), Rotation2d.kZero)
        ,new AllianceCoralKey(Alliance.Blue, Coral.Position_4),   new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.4)), Rotation2d.kZero)
    );

    // Currently active Field Correction Map

    public static Map<AllianceCoralKey, Transform2d> activeCorrectionMap = home;
}
