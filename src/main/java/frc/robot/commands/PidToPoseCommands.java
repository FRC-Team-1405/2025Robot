// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PidToPoseCommands {
  private static final double TOLERANCE = 1.2; // inches

  /* Debug Pose used for finding a position on the field in Advantage scope.
   * You can edit the location in network tables via shuffleboard
   */
  private static StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("DebugPose", Pose2d.struct).publish();

  /* Reef Poses */
  public static Pose2d Red_5A = new Pose2d(4.98, 2.84, Rotation2d.fromDegrees( -60));
  public static Pose2d Red_5B = new Pose2d(5.270, 3.000, Rotation2d.fromDegrees(-60));
  public static Pose2d Red_5B_AWAY = new Pose2d(4.8, 2, Rotation2d.fromDegrees(-60));
  public static Pose2d Red_6A = new Pose2d(3.71, 3, Rotation2d.fromDegrees(-120.0));
  public static Pose2d Red_6B = new Pose2d(3.99, 2.84, Rotation2d.fromDegrees(-120.0));

  public static Pose2d Blue_2A = new Pose2d(3.99, 5.21, Rotation2d.fromDegrees(120));
  public static Pose2d Blue_2B = new Pose2d(3.71, 5.05, Rotation2d.fromDegrees( 120));
  public static Pose2d Blue_3A = new Pose2d(5.27, 5.05, Rotation2d.fromDegrees(60));

  /* Feeder Station Poses */
  public static Pose2d LeftFeeder = new Pose2d(0.98, 7.05, Rotation2d.fromDegrees(125.0));
  public static Pose2d RightFeeder = new Pose2d(0.98, 1, Rotation2d.fromDegrees(-125.0));

  public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
    posePublisher.set(new Pose2d(4.8, 2, Rotation2d.kZero));

    /* Commands */
    Command MoveTo_Red_5B = new PidToPoseCommand(drivetrain, Red_5B, TOLERANCE, true, 0);
    Command MoveAway_Red_5B = new PidToPoseCommand(drivetrain, Red_5B_AWAY, TOLERANCE*3, true, 1);
    Command MoveTo_Red_5A = new PidToPoseCommand(drivetrain, Red_5A, TOLERANCE, true, 0);
    Command MoveTo_Red_6B = new PidToPoseCommand(drivetrain, Red_6B, TOLERANCE, true, 0);
    Command MoveTo_Red_6A = new PidToPoseCommand(drivetrain, Red_6A, TOLERANCE, true, 0);
    
    Command MoveTo_Blue_2A = new PidToPoseCommand(drivetrain, Blue_2A, TOLERANCE, true, 0);
    Command MoveTo_Blue_2B = new PidToPoseCommand(drivetrain, Blue_2B, TOLERANCE, true, 0);
    Command MoveTo_Blue_3A = new PidToPoseCommand(drivetrain, Blue_3A, TOLERANCE, true, 0);

    // Old Commands
    // Command MoveTo_Red_5B = drivetrain.runPidToPose(Red_5B, TOLERANCE, true, 0);
    // Command MoveAway_Red_5B = drivetrain.runPidToPose(Red_5B_AWAY, TOLERANCE, true, 1);
    // Command MoveTo_Red_5A = drivetrain.runPidToPose(Red_5A, TOLERANCE, true, 0);
    // Command MoveTo_Red_6B = drivetrain.runPidToPose(Red_6B, TOLERANCE, true, 0);
    // Command MoveTo_Red_6A = drivetrain.runPidToPose(Red_6A, TOLERANCE, true, 0);

    // Command MoveTo_Blue_2A = drivetrain.runPidToPose(Blue_2A, TOLERANCE, true, 0);
    // Command MoveTo_Blue_2B = drivetrain.runPidToPose(Blue_2B, TOLERANCE, true, 0);
    // Command MoveTo_Blue_3A = drivetrain.runPidToPose(Blue_3A, TOLERANCE, true, 0);

    /* Full Autos */
    Command P2P_DS_Right_3Piece = MoveTo_Red_5B.andThen(MoveAway_Red_5B);

    /* Register Commands */
    NamedCommands.registerCommand("MoveTo_Red_5B", MoveTo_Red_5B);
    NamedCommands.registerCommand("MoveAway_Red_5B", MoveAway_Red_5B);

    NamedCommands.registerCommand("MoveTo_Red_5A", MoveTo_Red_5A);
    NamedCommands.registerCommand("MoveTo_Red_6B", MoveTo_Red_6B);
    NamedCommands.registerCommand("MoveTo_Red_6A", MoveTo_Red_6A);

    NamedCommands.registerCommand("MoveTo_Blue_2A", MoveTo_Blue_2A);
    NamedCommands.registerCommand("MoveTo_Blue_2B", MoveTo_Blue_2B);
    NamedCommands.registerCommand("MoveTo_Blue_3A", MoveTo_Blue_3A);

    NamedCommands.registerCommand("P2P_DS_Right_3Piece", P2P_DS_Right_3Piece);
  }
}
