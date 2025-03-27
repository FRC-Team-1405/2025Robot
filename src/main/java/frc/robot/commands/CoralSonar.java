// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralSonar extends Command {
  private final Supplier<Pose2d> robotPose;
  private final List<Pose2d> redCoral;
  private final List<Pose2d> blueCoral;
  private final CommandXboxController controller;
  private Alliance alliance = Alliance.Blue;
  private double left = 6.5;    // distance in inches to the left of AprialTag
  private double right = -6.5;  // distance in inches to the right of AprialTag
  private double range = 2;     // range in inches of the target

  /** Creates a new CoralSonar. */
  public CoralSonar(Supplier<Pose2d> robotPose, CommandXboxController controller) {
    this.robotPose = robotPose;

    SmartDashboard.putNumber("CoralSonar/Left", 6.5);
    SmartDashboard.putNumber("CoralSonar/Right", -6.5);
    SmartDashboard.putNumber("CoralSonar/Range", 2.5);

    SmartDashboard.putNumber("CoralSonar/Target", 0);

    this.controller = controller;

    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    blueCoral = field.getTags().stream().filter(tag -> 17 <= tag.ID && tag.ID <= 22).map(tag -> tag.pose.toPose2d()).collect(Collectors.toList());

    field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    redCoral = field.getTags().stream().filter(tag -> 6 <= tag.ID && tag.ID <= 11).map(tag -> tag.pose.toPose2d()).collect(Collectors.toList());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.getAlliance().ifPresent(a -> alliance = a);
    left = SmartDashboard.getNumber("CoralSonar/Left", 6.5);
    right = SmartDashboard.getNumber("CoralSonar/Right", -6.5);
    range = SmartDashboard.getNumber("CoralSonar/Range", 2.5);

    SmartDashboard.putNumber("CoralSonar/Target", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d closestPose2d = robotPose.get().nearest( alliance == Alliance.Blue ? blueCoral : redCoral);
    Pose2d relative = robotPose.get().relativeTo(closestPose2d);

    double position = Units.metersToInches(relative.getY());
    SmartDashboard.putNumber("CoralSonar/Target", position);

    controller.setRumble(RumbleType.kBothRumble, 1.0);

    if ( Math.abs(position - left) < range || Math.abs(position - right) < range) {
      controller.setRumble(RumbleType.kBothRumble, 1.0);
    } else if (position < Math.min(left, right)-range) {
      controller.setRumble(RumbleType.kLeftRumble, 1.0);
    } else if (position > Math.min(left,right)+range) {
      controller.setRumble(RumbleType.kRightRumble, 1.0);
    } else {
      controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
