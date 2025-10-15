package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.lib.ReefSelecter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.RobotContainer.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

public class AutoPilotCommands {

    public static final SwerveRequest.ApplyFieldSpeeds pidToPose_FieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);

      private static final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(2.2, 0, 0); /* change these values for your robot */

    private static final APConstraints kConstraints = new APConstraints()
    .withAcceleration(7.0) // example was 5
    .withJerk(3); // example was 2

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Inches.of(1.5))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);

    // private static final TrapezoidProfile.Constraints DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 5);
    // private static ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, DEFAULT_CONSTRAINTS);


    // Rotations
    // Positive rotations are CCW: https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
    public static final Rotation2d kCW_30deg = Rotation2d.fromDegrees(30); // +30 is CCW but for some reason this is actually causing a CW rotation......can't figure out why
    public static Command autopilot(Supplier<Pose2d> target, boolean flipPoseForAlliance) {
        return autopilot(target, flipPoseForAlliance, Optional.empty());
    }

    /**
     * 
     * @param target
     * @param flipPoseForAlliance flip if red alliance
     * @param entryAngle Robot relative angle!
     * @return
     */
    public static Command autopilot(Supplier<Pose2d> target, boolean flipPoseForAlliance, Optional<Rotation2d> entryAngle) {
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return drivetrain.run(() -> {
            Pose2d correctedTargetPose = target.get();
            Optional<Rotation2d> correctedEntryAngle = entryAngle;
            if(flipPoseForAlliance && DriverStation.getAlliance().get().equals(Alliance.Red)) {
                correctedTargetPose = AllianceSymmetry.flip(correctedTargetPose);
            }

            if (correctedEntryAngle.isPresent()) {
                correctedEntryAngle = Optional.of( correctedEntryAngle.get().plus(correctedTargetPose.getRotation()) ); // convert from input of robot relative entry angle to field relative entry angle
            }

            ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;
            Pose2d currentPose = drivetrain.getState().Pose;
          
            APResult output;
            if (entryAngle.isPresent()){
                output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, new APTarget(correctedTargetPose).withEntryAngle(correctedEntryAngle.get()));
            } else {
                output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, new APTarget(correctedTargetPose).withoutEntryAngle());
            }
            
          
            /* these speeds are field relative */
            LinearVelocity veloX = output.vx();
            LinearVelocity veloY = output.vy();
            Rotation2d headingReference = output.targetAngle();

            // double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(),
            //     targetPose.getRotation().getRadians());
          
            /* This is where you should apply these speeds to the drivetrain */
            drivetrain.setControl(m_request
                .withVelocityX(veloX)
                .withVelocityY(veloY)
                .withTargetDirection(headingReference));
          }).until( () -> {
                Pose2d targetPose = target.get();
                if(flipPoseForAlliance && DriverStation.getAlliance().get().equals(Alliance.Red)) {
                    targetPose = AllianceSymmetry.flip(targetPose);
                }

                return kAutopilot.atTarget(drivetrain.getState().Pose, new APTarget(targetPose));
            }
        ).andThen( () -> {
                // thetaController.reset(0);
                drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            }
        );
    }

    public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
        /* Commands */
        // Uses command suppliers instead of commands so that we can reuse the same command in an autonomous
        ReefSelecter rs = RobotContainer.reefSelecter;
        Supplier<Command> MoveTo_Reef2       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_2).get(), false);
        // Supplier<Command> MoveAway_Reef2     = () -> new PidToPoseCommand(drivetrain, Reef_2_AWAY, 24, true, 0, 2, "MoveAway_Reef2", drivingContraints);
        // Supplier<Command> MoveAway_Reef2     = () -> autopilot(PidToPoseCommands.RightFeeder, true, Optional.of(Rotation2d.kCW_90deg.plus(kCW_30deg)));
        Supplier<Command> MoveAway_Reef2     = () -> autopilot(() -> new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(-125.0)), true, Optional.of(Rotation2d.kCW_90deg.plus(kCW_30deg)));
        Supplier<Command> MoveTo_Reef5       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_5).get(), false);
        Supplier<Command> MoveTo_Reef4       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_4).get(), false);

        Supplier<Command> MoveTo_Reef11       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_11).get(), false);
        Supplier<Command> MoveAway_Reef11     = () -> autopilot(PidToPoseCommands.Reef_11_AWAY, true); // high end velocity helps maintain momentum
        Supplier<Command> MoveTo_Reef9       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_9).get(), false);
        Supplier<Command> MoveTo_Reef8       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_8).get(), false);

        Supplier<Command> MoveTo_Reef12       = () -> autopilot(() -> rs.getRobotPositionForCoral(ReefSelecter.Coral.Position_12).get(), false);

        Supplier<Command> MoveTo_LeftFeeder   = () -> autopilot(PidToPoseCommands.LeftFeeder, true);
        Supplier<Command> MoveTo_LeftFeeder_InitialVel   = () -> autopilot(PidToPoseCommands.LeftFeeder, true);

        // Supplier<Command> MoveTo_RightFeeder  = () -> autopilot(PidToPoseCommands.RightFeeder, true);
        Supplier<Command> MoveTo_RightFeeder  = () -> autopilot(() -> new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(-125.0)), true); // todo remove this when you fix ap
        Supplier<Command> MoveTo_RightFeeder_InitialVel  = () -> autopilot(PidToPoseCommands.RightFeeder, true);

        /* Full Autos */
        Command AP_DS_Right_3Piece_WaitIntake = new SequentialCommandGroup(
            new ParallelCommandGroup(
                MoveTo_Reef2.get()
                // ,NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO), // todo remove when you fix ap
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            
            new ParallelCommandGroup(
                // new SequentialCommandGroup(
                MoveAway_Reef2.get(),
                // new ParallelDeadlineGroup(
                //     new WaitCommand(2.5),
                //     MoveTo_RightFeeder.get()
                // )
                // ),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
            ),
            NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

            new ParallelCommandGroup(
                MoveTo_Reef4.get()
                // ,NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO), // todo remove when you fix ap
            NamedCommands.getCommand(RobotContainer.OUTPUT_CORAL),
            
            new ParallelDeadlineGroup(
                new WaitCommand(2.5),
                MoveTo_RightFeeder.get(),
                NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_HOME)
            ),
            NamedCommands.getCommand(IntakeCommands.INTAKE_CORAL),

            new ParallelCommandGroup(
                MoveTo_Reef5.get()
                // ,NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO)
            ),
            NamedCommands.getCommand(RobotContainer.ELEVATOR_TO_LEVEL_4_AUTO), // todo remove when you fix ap
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

        Command AP_DS_Left_3Piece_ParallelIntake = new SequentialCommandGroup(
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
            Command AP_DS_Center = new SequentialCommandGroup(
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

        NamedCommands.registerCommand("AP_DS_Right_3Piece_WaitIntake", AP_DS_Right_3Piece_WaitIntake);
        NamedCommands.registerCommand("AP_DS_Left_3Piece_WaitIntake", AP_DS_Left_3Piece_WaitIntake);

        NamedCommands.registerCommand("AP_DS_Right_3Piece_ParallelIntake", AP_DS_Right_3Piece_ParallelIntake);

        NamedCommands.registerCommand("AP_DS_Left_3Piece_ParallelIntake", AP_DS_Left_3Piece_ParallelIntake);

        NamedCommands.registerCommand("AP_DS_Center", AP_DS_Center);
    }
}
