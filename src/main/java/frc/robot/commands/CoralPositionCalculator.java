// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPositionCalculator extends Command {
  private final Supplier<Pose2d> robotPose;
  private final List<Pose2d> aprilTagPositions;
  private final List<Pose2d> leftCoralPositions = new ArrayList<>();
  private final List<Pose2d> rightCoralPositions = new ArrayList<>();

  private final List<Pose2d> leftCoralRobotPositions = new ArrayList<>();
  private final List<Pose2d> rightCoralRobotPositions = new ArrayList<>();

  private Alliance alliance = Alliance.Blue;

  private double distanceToTarget = 0.0;


  private final double targetLeft = Units.inchesToMeters( -6.5 );
  private final double targetRight = Units.inchesToMeters( 6.5 );
  private final double targetRange = Units.inchesToMeters( 2.0 );

  private final double halfRobotLength = Units.inchesToMeters(17.5);

  StructArrayPublisher<Pose2d> leftCoralPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("LeftCoralPositions", Pose2d.struct).publish();

  StructArrayPublisher<Pose2d> rightCoralPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("RightCoralPositions", Pose2d.struct).publish();

  StructArrayPublisher<Pose2d> leftCoralRobotPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("LeftCoralRobotPositions", Pose2d.struct).publish();

  StructArrayPublisher<Pose2d> rightCoralRobotPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("RightCoralRobotPositions", Pose2d.struct).publish();

  /** Creates a new CoralPositionCalculator. */
  public CoralPositionCalculator(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;

    SmartDashboard.putNumber("CoralPositionCalculator/Target", 0);

    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    Transform2d left = new Transform2d(0, targetLeft, Rotation2d.kZero);
    Transform2d right = new Transform2d(0, targetRight, Rotation2d.kZero);
    Transform2d halfRobotTransform = new Transform2d(halfRobotLength, 0, Rotation2d.kZero);

    field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    aprilTagPositions = field.getTags()
      .stream()
      .filter(tag -> (17 <= tag.ID && tag. ID <= 22) || (6 <= tag.ID && tag.ID <= 11))
      .map(tag -> tag.pose.toPose2d())
      .collect(Collectors.toList());

    aprilTagPositions.stream().forEach(p -> leftCoralPositions.add( p.transformBy(left) ));
    aprilTagPositions.stream().forEach(p -> rightCoralPositions.add( p.transformBy(right) ));

    leftCoralPositions.stream().forEach(p -> leftCoralRobotPositions.add( p.transformBy(halfRobotTransform) ));
    rightCoralPositions.stream().forEach(p -> rightCoralRobotPositions.add( p.transformBy(halfRobotTransform) ));

    leftCoralRobotPositions.forEach( p -> {
      System.out.printf("Left Coral Robot Position %s\n", p.toString());
    });

    rightCoralRobotPositions.forEach( p -> {
      System.out.printf("Right Coral Robot Position %s\n", p.toString());
    });

    visualizePoints(leftCoralPublisher, leftCoralPositions);
    visualizePoints(rightCoralPublisher, rightCoralPositions);

    visualizePoints(leftCoralRobotPublisher, leftCoralRobotPositions);
    visualizePoints(rightCoralRobotPublisher, rightCoralRobotPositions);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.getAlliance().ifPresent(a -> alliance = a);

    distanceToTarget = 0.0;
    SmartDashboard.putNumber("CoralPositionCalculator/Target", distanceToTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = 0.0;
    Pose2d robot = robotPose.get();

    aprilTagPositions
      .stream()
      .map(p -> robot.relativeTo(p).getY())
      .min(Comparator.comparing(d -> Math.abs(d)))
      .ifPresent(d -> distanceToTarget = d);

    SmartDashboard.putNumber("CoralPositionCalculator/Target", distance);
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(distanceToTarget) < targetRange;
    return true;
  }

  /**
   * Publish points to smart dashboard so you can visualize them.
   */
  private void visualizePoints(StructArrayPublisher<Pose2d> publisher, List<Pose2d> points) {
    Pose2d[] array = new Pose2d[points.size()];
    points.toArray(array);

    publisher.set(array);
  }
}
