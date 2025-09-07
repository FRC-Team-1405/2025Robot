// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.ReefSelecter;
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
  public static Supplier<Pose2d> Red_5B_AWAY  = () -> new Pose2d(4.8, 1, Rotation2d.fromDegrees(-60));

  /* Feeder Station Poses from perspective of DS */
  public static Supplier<Pose2d> LeftFeeder   = () -> new Pose2d(0.98, 7.05, Rotation2d.fromDegrees(125.0));
  public static Supplier<Pose2d> RightFeeder  = () -> new Pose2d(0.98, 1, Rotation2d.fromDegrees(-125.0));

  public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
    posePublisher.set(new Pose2d(4.8, 2, Rotation2d.kZero));

    /* Commands */
    // Uses command suppliers instead of commands so that we can reuse the same command in an autonomous
    ReefSelecter rs = RobotContainer.reefSelecter;
    Supplier<Command> MoveTo_Reef2       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_2).get(), TOLERANCE, "MoveTo_Reef2");
    Supplier<Command> MoveAway_Reef2     = () -> new PidToPoseCommand(drivetrain, Red_5B_AWAY, 24, true, 0, 2, "MoveAway_Reef2");
    Supplier<Command> MoveTo_Reef5       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_5).get(), TOLERANCE, "MoveTo_Reef5");
    Supplier<Command> MoveTo_Reef4       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_4).get(), TOLERANCE, "MoveTo_Reef4");

    Supplier<Command> MoveTo_LeftFeeder   = () -> new PidToPoseCommand(drivetrain, LeftFeeder, TOLERANCE, true, "MoveTo_LeftFeeder");
    Supplier<Command> MoveTo_RightFeeder  = () -> new PidToPoseCommand(drivetrain, RightFeeder, TOLERANCE, true, "MoveTo_RightFeeder");
    Supplier<Command> MoveTo_RightFeeder_InitialVel  = () -> new PidToPoseCommand(drivetrain, RightFeeder, TOLERANCE, true, 2, 0, "MoveTo_RightFeeder_InitialVel");

    /* Full Autos */
    Command P2P_DS_Right_3Piece = new SequentialCommandGroup(
      new ParallelCommandGroup(
        MoveTo_Reef2.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      MoveAway_Reef2.get(),
      
      new ParallelCommandGroup(
        MoveTo_RightFeeder_InitialVel.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),
      NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

      new ParallelCommandGroup(
        MoveTo_Reef4.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        MoveTo_RightFeeder.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),
      NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

      new ParallelCommandGroup(
        MoveTo_Reef5.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL)
    );

    /* Register Commands */
    NamedCommands.registerCommand("MoveTo_Reef2", MoveTo_Reef2.get());
    NamedCommands.registerCommand("MoveAway_Reef2", MoveAway_Reef2.get());

    NamedCommands.registerCommand("MoveTo_Reef5", MoveTo_Reef5.get());
    NamedCommands.registerCommand("MoveTo_Reef4", MoveTo_Reef4.get());

    NamedCommands.registerCommand("P2P_DS_Right_3Piece", P2P_DS_Right_3Piece);
  }
}
