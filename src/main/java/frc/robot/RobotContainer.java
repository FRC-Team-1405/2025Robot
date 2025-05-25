// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.CoralInput;
import frc.robot.commands.CoralOutput;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.PidToPoseCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.ReefSelecter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top speed
  public static final double MaxAngularRate = 11.22;// in radians per second
  
  // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public static final SwerveRequest.FieldCentric pidToPose_FieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);


  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  private final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Vision vision = new Vision(Vision.camerasFromConfigs(VisionConstants.CONFIGS));

  /*
   * Named Commands Constants
   */

  private final String ELEVATOR_TO_LEVEL_4_AUTO = "Elevator to Level4 Auto";
  private final String SCORE_LEVEL_4_CORAL = "Score Level4 Coral";
  private final String ELEVATOR_TO_LEVEL_4 = "Elevator To Level4";
  private final String ELEVATOR_TO_SELECTED_LEVEL = "Elevator To Selected Level";
  private final String ELEVATOR_TO_HOME = "Elevator To Home";
  private final String OUTPUT_CORAL = "Output Coral";

  // region FeatureSwitches
  public static final boolean AMBIGUITY_FILTER = true;
  public static final boolean LONG_DISTANCE_FILTER = true;
  public static final boolean RESET_CAMERA_RESULTS = false;
  public static final boolean VISION_ODOMETRY_ESTIMATION = true; // Enable vision and publish its estimated position
                                                                 // (doesn't update robot odometry)
  public static boolean VISION_ROBOT_ODOMETRY_UPDATE = true; // Enable vision odometry updates while driving. Doesn't
                                                             // work without VISION_ODOMETRY_ESTIMATION set to true.
  // endregion FeatureSwitches

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configurePathPlanner();
    autoChooser = AutoBuilder.buildAutoChooser("DriveStraight3m");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
    drivetrain.configureShuffleboardCommands();

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y
                                                                                           // (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                        // negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.b().onTrue(drivetrain.runOnce(() -> {
      drivetrain.resetPose(new Pose2d(1, 1, new Rotation2d(0)));
    }));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));



    Pose2d pidToPoseTarget = new Pose2d(5.270, 3.000, Rotation2d.fromDegrees(-60));
    joystick.a().whileTrue(
      CommandSwerveDrivetrain.PidToPose(drivetrain, pidToPoseTarget, 1)
      .alongWith(
        Commands.sequence(
          Commands.waitUntil(() -> drivetrain.getState().Pose.getTranslation().getDistance(pidToPoseTarget.getTranslation()) < 1)),
          new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake)
        )
      .andThen(
        new ParallelRaceGroup(
            new CoralOutput(intake),
            new ArmPosition(elevator, () -> ArmLevel.Travel).beforeStarting(Commands.waitSeconds(0.25)))
      )
      ).onFalse(new MoveCoral(elevator, () -> ElevationLevel.Home, intake));



    joystick.b().whileTrue(drivetrain.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    joystick.x().whileTrue(drivetrain
        .driveToPose(new Pose2d()));
    joystick.y()
        .whileTrue(drivetrain
            .driveToPose(
                () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition())));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));

    joystick.rightBumper().toggleOnTrue(new CoralInput(intake));
    joystick.leftBumper()
        .onTrue(new SequentialCommandGroup(new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel)));
    // driver.a().onTrue(new SequentialCommandGroup(new ArmPosition(elevator, () ->
    // ArmLevel.Climb)));
    // driver.back().onTrue((Commands.runOnce(driveBase::zeroGyroWithAlliance)).ignoringDisable(true));

    if (false) {

      // Driver presses AND HOLDS B to activate auto align. auto align will move to
      // the scoring position while raising the elevator.
      // It will not score the coral, the operator will need to output the coral.
      // when the driver lets go the auto align will stop. the elevator will not move
      // until operator moves it.
      if (false) {
        joystick.b()
            .whileTrue(drivetrain
                .driveToPose(
                    () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition()))
                .alongWith(NamedCommands.getCommand(ELEVATOR_TO_SELECTED_LEVEL)));
      } else {
        joystick.b()
            .whileTrue(drivetrain
                .driveToPose(
                    () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition()))
                .alongWith(NamedCommands.getCommand(ELEVATOR_TO_LEVEL_4)));
      }

    } else {

      // Driver presses AND HOLDS B to activate auto align. auto align will move to
      // the scoring position, raise the elevator and score the coral in sequence.
      // when the driver lets go the auto align will stop. the elevator will not move
      // until operator moves it.
      if (false) {
        joystick.b()
            .whileTrue(drivetrain
                .driveToPose(
                    () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition()))
                .andThen(
                    NamedCommands.getCommand(ELEVATOR_TO_SELECTED_LEVEL)));
      } else {
        joystick.b()
            .whileTrue(drivetrain
                .driveToPose(
                    () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition()))
                .andThen(
                    NamedCommands.getCommand(ELEVATOR_TO_LEVEL_4)));
      }
    }

    operator.y().onTrue(new MoveCoral(elevator, reefSelecter::getLevel, intake));
    operator.a().onTrue(new MoveCoral(elevator, () -> ElevationLevel.Home, intake));
    operator.x().onTrue(new InstantCommand(() -> {
      intake.outtakeCoral();
    }));
    operator.x().onFalse(new InstantCommand(() -> {
      intake.stop();
    }));

    operator.povLeft().onTrue(Commands.runOnce(reefSelecter::selectLeft));
    operator.povRight().onTrue(Commands.runOnce(reefSelecter::selectRight));

    operator.povUp()
        .or(operator.povUpLeft())
        .or(operator.povUpRight())
        .onTrue(new InstantCommand(() -> {
          reefSelecter.levelUp();
        }));

    operator.povDown()
        .or(operator.povDownLeft())
        .or(operator.povDownRight())
        .onTrue(new InstantCommand(() -> {
          reefSelecter.levelDown();
        }));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
    // return null;
  }

  void configurePathPlanner() {

    NamedCommands.registerCommand(ELEVATOR_TO_LEVEL_4,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake)));
    NamedCommands.registerCommand(ELEVATOR_TO_SELECTED_LEVEL,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> reefSelecter.getLevel(), intake)));

    NamedCommands.registerCommand(ELEVATOR_TO_HOME,
        new MoveCoral(elevator, () -> ElevationLevel.Home, intake));

    NamedCommands.registerCommand(OUTPUT_CORAL,
        new ParallelRaceGroup(
            new CoralOutput(intake),
            new ArmPosition(elevator, () -> ArmLevel.Travel).beforeStarting(Commands.waitSeconds(0.25))));

    NamedCommands.registerCommand(SCORE_LEVEL_4_CORAL,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake),
            new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveCoral(elevator, () -> ElevationLevel.Home, intake)));
    NamedCommands.registerCommand("Score Level3 Coral",
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_3, intake),
            new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveCoral(elevator, () -> ElevationLevel.Home, intake)));
    NamedCommands.registerCommand("Score Level2 Coral",
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_2, intake),
            new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveCoral(elevator, () -> ElevationLevel.Home, intake)));
    NamedCommands.registerCommand("Intake Coral", new CoralInput(intake));

    NamedCommands.registerCommand(ELEVATOR_TO_LEVEL_4_AUTO,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_4_Auto, intake)));

    PidToPoseCommands.registerCommands(drivetrain);
  }

  public void correctOdometry() {
    var visionSamples = vision.flushSamples();
    vision.updateSpeeds(drivetrain.getState().Speeds);
    for (var sample : visionSamples) {
      double thetaStddev = sample.weight() > 0.9 ? 25.0 : 99999.0;
      drivetrain.addVisionMeasurement(
        sample.pose(),
        sample.timestamp(),
        VecBuilder.fill(0.1 / sample.weight(), 0.1 / sample.weight(), thetaStddev)
      );
    }
  }
}
