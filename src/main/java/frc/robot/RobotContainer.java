// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Climb;
import frc.robot.commands.CoralInput;
import frc.robot.commands.CoralOutput;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.LowScore;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.NewCoralInput;
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
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driver = new CommandXboxController(0);
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
  private static final String NO_SELECTED_AUTO = "None";

  // region FeatureSwitches
  public static final boolean DEBUG_CONSOLE_LOGGING = false;
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
    configureShuffleboardCommands();
    drivetrain.configureShuffleboardCommands();

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private boolean highAlgae = true;

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(getXSpeed()) // Drive forward with
                                                                                           // negative Y
                                                                                           // (forward)
            .withVelocityY(getYSpeed()) // Drive left with negative X (left)
            .withRotationalRate(getRotationSpeed()) // Drive counterclockwise with
                                                                        // negative X (left)
        ));
    
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    
    driver.rightBumper().toggleOnTrue(new CoralInput(intake));
    driver.leftBumper()
        .onTrue(new SequentialCommandGroup(new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel)));
    driver.a().onTrue(new SequentialCommandGroup(new ArmPosition(elevator, () -> ArmLevel.Climb)));
    
    /* B Button: Auto Align  */
    driver.b().whileTrue(
      drivetrain.runAutoAlign(() -> reefSelecter.getRobotPositionForSelectedCoral(), reefSelecter::getLevel, intake, elevator)
    ).onFalse(new MoveCoral(elevator, () -> ElevationLevel.Home, intake));

    driver.povUp().toggleOnTrue(new NewCoralInput(intake));

    driver.back().onTrue(
      Commands.runOnce(() -> {
        double angleToResetTo = DriverStation.Alliance.Blue.equals(DriverStation.getAlliance().get()) ? 0 : 180;
        drivetrain.resetRotation(new Rotation2d(Units.degreesToRadians(angleToResetTo)));
      }
      ).ignoringDisable(true));

    

    SmartDashboard.putBoolean("Algae/High", highAlgae);
    SmartDashboard.putBoolean("Algae/Low", !highAlgae);
    operator.rightBumper().onTrue(Commands.runOnce(() -> {
      highAlgae = true;
      SmartDashboard.putBoolean("Algae/High", highAlgae);
      SmartDashboard.putBoolean("Algae/Low", !highAlgae);
    }));
    operator.leftBumper().onTrue(Commands.runOnce(() -> {
      highAlgae = false;
      SmartDashboard.putBoolean("Algae/High", highAlgae);
      SmartDashboard.putBoolean("Algae/Low", !highAlgae);
    }));
    operator.b().onTrue(new GrabAlgae(elevator, intake, () -> {
      return highAlgae;
    }));

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

    Command climbCommand = new Climb(climber, () -> {
      return operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
    });
    climbCommand.setName("Climb Command");
    SmartDashboard.putData(climbCommand);

    operator.start().and(operator.back()).toggleOnTrue(climbCommand);

    drivetrain.registerTelemetry(logger::telemeterize);
  }


  private void oldConfigureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y
                                                                                           // (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                        // negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.b().onTrue(drivetrain.runOnce(() -> {
      drivetrain.resetPose(new Pose2d(1, 1, new Rotation2d(0)));
    }));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    /* A Button: Auto Align  */
    driver.a().whileTrue(
      drivetrain.runAutoAlign(() -> reefSelecter.getRobotPositionForSelectedCoral(), reefSelecter::getLevel, intake, elevator)
    ).onFalse(new MoveCoral(elevator, () -> ElevationLevel.Home, intake));

    driver.b().whileTrue(drivetrain.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));

    driver.rightBumper().toggleOnTrue(new CoralInput(intake));
    driver.leftBumper()
        .onTrue(new SequentialCommandGroup(new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel)));
    // driver.a().onTrue(new SequentialCommandGroup(new ArmPosition(elevator, () ->
    // ArmLevel.Climb)));
    // driver.back().onTrue((Commands.runOnce(driveBase::zeroGyroWithAlliance)).ignoringDisable(true));

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
    if (DriverStation.isFMSAttached() || SmartDashboard.getBoolean("Auto Mode Enable", false)) {
      SmartDashboard.putBoolean("Auto Mode Enable", false);
      return autoChooser.getSelected();
    } else {
      return Commands.print("Auto Disabled");
    }
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
        Commands.sequence(Commands.waitUntil(intake::hasCoral), new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake)).unless(() -> !intake.hasCoral()));

    PidToPoseCommands.registerCommands(drivetrain);
  }

  public void correctOdometry() {
    var visionSamples = vision.flushSamples();
    vision.updateSpeeds(drivetrain.getState().Speeds);
    for (var sample : visionSamples) {
      double thetaStddev = sample.weight() > 0.9 ? 10.0 : 99999.0;
      drivetrain.addVisionMeasurement(
        sample.pose(),
        sample.timestamp(),
        VecBuilder.fill(0.1 / sample.weight(), 0.1 / sample.weight(), thetaStddev)
      );
    }
  }

  private void configureShuffleboardCommands() {
    Command outputCoral = new CoralOutput(intake);
    outputCoral.setName("Output Coral");
    SmartDashboard.putData(outputCoral);

    Command inputCoral = new CoralInput(intake);
    inputCoral.setName("Input Coral");
    SmartDashboard.putData(inputCoral);

    Trigger reefTrigger = new Trigger(intake::reefDetected);
    reefTrigger.onTrue(outputCoral);

    Command zeroizeClimber = climber.runOnce(() -> {
      climber.zeroize();
    });
    zeroizeClimber.setName("zeroize Climber");
    SmartDashboard.putData(zeroizeClimber);
    SmartDashboard.putBoolean("Auto Mode Enable", false);

    Command lowScore = new LowScore(elevator, intake);
    lowScore.setName("Low Score");
    SmartDashboard.putData(lowScore);

    SmartDashboard.putNumber("Test/Short Drive Time", 0.5);
    SmartDashboard.putNumber("Test/Reef Drive", 2.0);
  }

  private double getYSpeed(){
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);    
    return -driver.getLeftX() * speedMultiplication * MaxSpeed;
  }

  private double getXSpeed(){
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);
    return -driver.getLeftY() * speedMultiplication * MaxSpeed;
  }

  public double getRotationSpeed() {
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);
    return -driver.getRightX() * speedMultiplication * MaxAngularRate;
  }
}
