// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralSonar extends Command {
  private final Supplier<Pose2d> robotPose;
  private final List<Pose2d> redCoral;
  private final List<Pose2d> blueCoral;
  private Alliance alliance = Alliance.Blue;

  private double distanceToTarget = 0.0;

  private final double targetLeft;
  private final double targetRight;
  private final double targetRange;

  /** Creates a new CoralSonar. */
  public CoralSonar(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;

    Preferences.initDouble("CoralSonar/LeftTarget", 6.5);
    Preferences.initDouble("CoralSonar/RightTarget", -6.5);
    Preferences.initDouble("CoralSonar/Range", 2.0);
    targetLeft = Units.inchesToMeters( Preferences.getDouble("CoralSonar/LeftTarget", 6.5) );
    targetRight = Units.inchesToMeters( Preferences.getDouble("CoralSonar", -6.5) );
    targetRange = Units.inchesToMeters( Preferences.getDouble("CoralSonar/Range", 2.0) );

    SmartDashboard.putNumber("CoralSonar/Target", 0);

    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    Transform2d left = new Transform2d(0, Units.inchesToMeters(targetLeft), Rotation2d.kZero);
    Transform2d right = new Transform2d(0, Units.inchesToMeters(targetRight), Rotation2d.kZero);

    field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    blueCoral = field.getTags()
      .stream()
      .filter(tag -> 17 <= tag.ID && tag.ID <= 22)
      .map(tag -> tag.pose.toPose2d())
      .flatMap(tag -> Arrays.asList(tag.transformBy(left), tag.transformBy(right)).stream())
      .collect(Collectors.toList());

    field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    redCoral = field.getTags()
      .stream()
      .filter(tag -> 6 <= tag.ID && tag.ID <= 11).map(tag -> tag.pose.toPose2d())
      .flatMap(tag -> Arrays.asList(tag.transformBy(left), tag.transformBy(right)).stream())
      .collect(Collectors.toList());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.getAlliance().ifPresent(a -> alliance = a);

    distanceToTarget = 0.0;
    SmartDashboard.putNumber("CoralSonar/Target", distanceToTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = 0.0;
    Pose2d robot = robotPose.get();

    (alliance == Alliance.Blue ? blueCoral : redCoral)
      .stream()
      .map(p -> robot.relativeTo(p).getY())
      .min(Comparator.comparing(d -> Math.abs(d)))
      .ifPresent(d -> distanceToTarget = d);

    SmartDashboard.putNumber("CoralSonar/Target", distance);
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(distanceToTarget) < targetRange;
  }
}
