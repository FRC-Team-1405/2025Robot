package frc.robot.lib.FieldCorrection;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.ReefSelecter.Coral;

public class FieldCorrectionMap {
    public record AllianceCoralKey (Alliance alliance, Coral coral) {}

    // List of Field Correction Maps

    private static Map<AllianceCoralKey, Transform2d> home = Map.of (
         new AllianceCoralKey(Alliance.Red, Coral.Position_2),   new Transform2d(new Translation2d(Units.inchesToMeters(2), Units.inchesToMeters(0)), Rotation2d.kZero)
        ,new AllianceCoralKey(Alliance.Blue, Coral.Position_2),   new Transform2d(new Translation2d(Units.inchesToMeters(2), Units.inchesToMeters(0)), Rotation2d.kZero)
    );

    // Currently active Field Correction Map

    public static Map<AllianceCoralKey, Transform2d> activeCorrectionMap = home;
}
