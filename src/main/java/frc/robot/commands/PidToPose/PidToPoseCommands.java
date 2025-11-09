// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PidToPose;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands;
import frc.robot.constants.FieldConstants.FeederSide;
import frc.robot.constants.FieldConstants.FeederStation;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.ReefSelecter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PidToPoseCommands {
  public static final double SCORE_TOLERANCE = 1.2; // inches
  private static final double TOLERANCE = 2; // inches

  private static final TrapezoidProfile.Constraints drivingContraints = new TrapezoidProfile.Constraints(5, 6);

  /* Debug Pose used for finding a position on the field in Advantage scope.
   * You can edit the location in network tables via shuffleboard
   */
  private static StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("DebugPose", Pose2d.struct).publish();

  /* Reef Poses */
  // public static Supplier<Pose2d> Reef_2_AWAY  = () -> new Pose2d(4.8, 1, Rotation2d.fromDegrees(-60));
  public static Supplier<Pose2d> Reef_2_AWAY  = () -> new Pose2d(4.67, 2.04, Rotation2d.fromDegrees(-60));
  public static Supplier<Pose2d> Reef_11_AWAY  = () -> new Pose2d(4.67, 6.01, Rotation2d.fromDegrees(60));

  /* Feeder Station Poses from perspective of DS, all positions are for the blue alliance. they are flipped later if used on red alliance */
  public static Supplier<Pose2d> LeftFeeder   = () -> FeederStation.getRetrievalPose(FeederSide.LEFT, RobotConstants.ROBOT_WIDTH);
  public static Supplier<Pose2d> RightFeeder  = () -> FeederStation.getRetrievalPose(FeederSide.RIGHT, RobotConstants.ROBOT_WIDTH);

  public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
    Pose2d debugPose = new Pose2d(4.22, 3.22, Rotation2d.fromDegrees(60));
    // Pose2d flippedPose = AllianceSymmetry.flip(new Pose2d(4.67, 2.04, Rotation2d.fromDegrees(-60)), SymmetryStrategy.HORIZONTAL);
    posePublisher.set(debugPose);

    /* Commands */
    // Uses command suppliers instead of commands so that we can reuse the same command in an autonomous
    ReefSelecter rs = RobotContainer.reefSelecter;
    Supplier<Command> MoveTo_Reef2       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_2).get(), SCORE_TOLERANCE, "MoveTo_Reef2");
    // Supplier<Command> MoveAway_Reef2     = () -> new PidToPoseCommand(drivetrain, Reef_2_AWAY, 24, true, 0, 2, "MoveAway_Reef2", drivingContraints);
    Supplier<Command> MoveAway_Reef2     = () -> new PidToPoseCommand(drivetrain, Reef_2_AWAY, 20, true, 0, 15, "MoveAway_Reef2", drivingContraints); // high end velocity helps maintain momentum
    Supplier<Command> MoveTo_Reef5       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_5).get(), SCORE_TOLERANCE, "MoveTo_Reef5");
    Supplier<Command> MoveTo_Reef4       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_4).get(), SCORE_TOLERANCE, "MoveTo_Reef4");

    Supplier<Command> MoveTo_Reef11       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_11).get(), SCORE_TOLERANCE, "MoveTo_Reef11");
    Supplier<Command> MoveAway_Reef11     = () -> new PidToPoseCommand(drivetrain, Reef_11_AWAY, 20, true, 0, 15, "MoveAway_Reef11", drivingContraints); // high end velocity helps maintain momentum
    Supplier<Command> MoveTo_Reef9       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_9).get(), SCORE_TOLERANCE, "MoveTo_Reef9");
    Supplier<Command> MoveTo_Reef8       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_8).get(), SCORE_TOLERANCE, "MoveTo_Reef8");

    Supplier<Command> MoveTo_Reef12       = () -> new PidToPoseCommand(drivetrain, () -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_12).get(), SCORE_TOLERANCE, "MoveTo_Reef12");

    Supplier<Command> MoveTo_LeftFeeder   = () -> new PidToPoseCommand(drivetrain, LeftFeeder, TOLERANCE, true, 0, 0, "MoveTo_LeftFeeder", drivingContraints);
    Supplier<Command> MoveTo_LeftFeeder_InitialVel   = () -> new PidToPoseCommand(drivetrain, LeftFeeder, TOLERANCE, true, 3, 0, "MoveTo_LeftFeeder_InitialVel", drivingContraints);

    Supplier<Command> MoveTo_RightFeeder  = () -> new PidToPoseCommand(drivetrain, RightFeeder, TOLERANCE, true, 0, 0, "MoveTo_RightFeeder", drivingContraints);
    Supplier<Command> MoveTo_RightFeeder_InitialVel  = () -> new PidToPoseCommand(drivetrain, RightFeeder, TOLERANCE, true, 3, 0, "MoveTo_RightFeeder_InitialVel", drivingContraints);

    /* Full Autos */
    Command P2P_DS_Right_3Piece_WaitIntake = new SequentialCommandGroup(
      new ParallelCommandGroup(
        MoveTo_Reef2.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          MoveAway_Reef2.get(),
          new ParallelDeadlineGroup(
            new WaitCommand(2.5),
            MoveTo_RightFeeder.get()
          )
        ),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),
      NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

      new ParallelCommandGroup(
        MoveTo_Reef4.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelDeadlineGroup(
        new WaitCommand(2.5),
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

    Command P2P_DS_Left_3Piece_WaitIntake = new SequentialCommandGroup(
      new ParallelCommandGroup(
        MoveTo_Reef11.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          MoveTo_Reef11.get(),
          MoveTo_LeftFeeder.get()
        ),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),
      NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

      new ParallelCommandGroup(
        MoveTo_Reef9.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        MoveTo_RightFeeder.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),
      NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

      new ParallelCommandGroup(
        MoveTo_Reef8.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL)
    );


    Command P2P_DS_Right_3Piece_ParallelIntake = new SequentialCommandGroup(
      new ParallelCommandGroup(
        MoveTo_Reef2.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          MoveAway_Reef2.get(),
          MoveTo_RightFeeder_InitialVel.get()
        ),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),

      new ParallelCommandGroup(
        MoveTo_Reef4.get(),
        new SequentialCommandGroup(
          NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),
          NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
        )
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        MoveTo_RightFeeder.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),

      new ParallelCommandGroup(
        MoveTo_Reef5.get(),
        new SequentialCommandGroup(
          NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),
          NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
        )
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL)
    );

    Command P2P_DS_Left_3Piece_ParallelIntake = new SequentialCommandGroup(
      new ParallelCommandGroup(
        MoveTo_Reef11.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          MoveAway_Reef11.get(),
          MoveTo_LeftFeeder_InitialVel.get()
        ),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),

      new ParallelCommandGroup(
        MoveTo_Reef9.get(),
        new SequentialCommandGroup(
          NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),
          NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
        )
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      
      new ParallelCommandGroup(
        MoveTo_LeftFeeder.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
      ),

      new ParallelCommandGroup(
        MoveTo_Reef8.get(),
        new SequentialCommandGroup(
          NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),
          NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
        )
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL)
    );

    // Score one coral in center position and then stop
    Command P2P_DS_Center = new SequentialCommandGroup(
      new ParallelCommandGroup(
        MoveTo_Reef12.get(),
        NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
      ),
      NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
      NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
    );

    /* Register Commands */
    // NamedCommands.registerCommand("MoveTo_Reef2", MoveTo_Reef2.get());
    // NamedCommands.registerCommand("MoveAway_Reef2", MoveAway_Reef2.get());

    // NamedCommands.registerCommand("MoveTo_Reef5", MoveTo_Reef5.get());
    // NamedCommands.registerCommand("MoveTo_Reef4", MoveTo_Reef4.get());

    NamedCommands.registerCommand("P2P_DS_Right_3Piece_WaitIntake", P2P_DS_Right_3Piece_WaitIntake);
    NamedCommands.registerCommand("P2P_DS_Left_3Piece_WaitIntake", P2P_DS_Left_3Piece_WaitIntake);

    NamedCommands.registerCommand("P2P_DS_Right_3Piece_ParallelIntake", P2P_DS_Right_3Piece_ParallelIntake);

    NamedCommands.registerCommand("P2P_DS_Left_3Piece_ParallelIntake", P2P_DS_Left_3Piece_ParallelIntake);

    NamedCommands.registerCommand("P2P_DS_Center", P2P_DS_Center);
  }
}
