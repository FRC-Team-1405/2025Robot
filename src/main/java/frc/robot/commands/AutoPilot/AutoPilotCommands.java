package frc.robot.commands.AutoPilot;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class AutoPilotCommands {

    // Rotations
    // Positive rotations are CCW: https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
    public static final Rotation2d CW_30deg = Rotation2d.fromDegrees(-30);
    public static final Rotation2d CCW_30deg = Rotation2d.fromDegrees(30);

    /* Feeder Station Poses from perspective of DS, all positions are for the blue alliance. they are flipped later if used on red alliance */
    public static Supplier<Pose2d> LeftFeeder   = () -> FeederStation.getRetrievalPose(FeederSide.LEFT, RobotConstants.ROBOT_WIDTH);
    public static Supplier<Pose2d> RightFeeder  = () -> FeederStation.getRetrievalPose(FeederSide.RIGHT, RobotConstants.ROBOT_WIDTH);

    public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
        /* Commands */
        // Uses command suppliers instead of commands so that we can reuse the same command in an autonomous
        ReefSelecter rs = RobotContainer.reefSelecter;

        // Right
        Supplier<Command> MoveTo_Reef2       = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_2).get(), drivetrain, "MoveTo_Reef2");
        Supplier<Command> MoveTo_Reef5       = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_5).get(), drivetrain, "MoveTo_Reef5");
        Supplier<Command> MoveTo_Reef4       = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_4).get(), drivetrain, "MoveTo_Reef4");

        // Left
        Supplier<Command> MoveTo_Reef11      = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_11).get(), drivetrain, "MoveTo_Reef11");
        Supplier<Command> MoveTo_Reef9       = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_9).get(), drivetrain, "MoveTo_Reef9");
        Supplier<Command> MoveTo_Reef8       = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_8).get(), drivetrain, "MoveTo_Reef8");

        // Center
        Supplier<Command> MoveTo_Reef12       = () -> new AutoPilotCommand(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_12).get(), drivetrain, "MoveTo_Reef12");

        // Feeders
        Supplier<Command> MoveTo_LeftFeeder                = () -> new AutoPilotCommand(LeftFeeder, drivetrain, Optional.empty(), true, "MoveTo_LeftFeeder");
        Supplier<Command> MoveTo_LeftFeeder_EntryAngle     = () -> new AutoPilotCommand(LeftFeeder, drivetrain, Optional.of(CCW_30deg.minus(CW_30deg)), true, "MoveTo_LeftFeeder_EntryAngle");

        Supplier<Command> MoveTo_RightFeeder                = () -> new AutoPilotCommand(RightFeeder, drivetrain, Optional.empty(), true, "MoveTo_RightFeeder");
        Supplier<Command> MoveTo_RightFeeder_EntryAngle     = () -> new AutoPilotCommand(RightFeeder, drivetrain, Optional.of(CW_30deg.plus(CW_30deg)), true, "MoveTo_RightFeeder_EntryAngle");

        /* Full Autos */
        Command AP_DS_Right_3Piece_WaitIntake = new SequentialCommandGroup(
            new ParallelCommandGroup(
                MoveTo_Reef2.get()
                ,NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),

            // Score first coral
            // NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO), // todo remove when you fix ap
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            
            new ParallelCommandGroup(
                // new SequentialCommandGroup(
                MoveTo_RightFeeder_EntryAngle.get(),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
            ),
            NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

            new ParallelCommandGroup(
                MoveTo_Reef4.get()
                ,NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            // NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO), // todo remove when you fix ap
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            
            new ParallelCommandGroup(
                // new ParallelDeadlineGroup(
                // new WaitCommand(2.5), TODO was breaking simulation, when simulation elevator is fixed readd
                MoveTo_RightFeeder.get(),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
            ),
            NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

            new ParallelCommandGroup(
                MoveTo_Reef5.get()
                ,NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            // NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO), // todo remove when you fix ap
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL)
        );

        Command AP_DS_Left_3Piece_WaitIntake = new SequentialCommandGroup(
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


        Command AP_DS_Right_3Piece_ParallelIntake = new SequentialCommandGroup(
            new ParallelCommandGroup(
                MoveTo_Reef2.get(),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                MoveTo_RightFeeder_EntryAngle.get()
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

        Command AP_DS_Left_3Piece_ParallelIntake = new SequentialCommandGroup(
            new ParallelCommandGroup(
                MoveTo_Reef11.get(),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    MoveTo_LeftFeeder_EntryAngle.get()
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
        Command AP_DS_Center = new SequentialCommandGroup(
            new ParallelCommandGroup(
                MoveTo_Reef12.get(),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
        );

        /* Register Commands */
        NamedCommands.registerCommand("AP_DS_Right_3Piece_WaitIntake", AP_DS_Right_3Piece_WaitIntake);
        NamedCommands.registerCommand("AP_DS_Left_3Piece_WaitIntake", AP_DS_Left_3Piece_WaitIntake);

        NamedCommands.registerCommand("AP_DS_Right_3Piece_ParallelIntake", AP_DS_Right_3Piece_ParallelIntake);
        NamedCommands.registerCommand("AP_DS_Left_3Piece_ParallelIntake", AP_DS_Left_3Piece_ParallelIntake);

        NamedCommands.registerCommand("AP_DS_Center", AP_DS_Center);
    }
}
