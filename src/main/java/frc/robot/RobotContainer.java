// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.autos.TestAuto;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Climb;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.PidToPoseCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.lib.ReefSelecter;
import frc.robot.lib.AllianceSymmetry.SymmetryStrategy;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionSample;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top speed
  public static final double MaxAngularRate = 11.22;// in radians per second
  
  // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.07).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);


  // Vision publishers
  StructPublisher<Pose2d> cameraEstimatedPosePublisher1 = NetworkTableInstance.getDefault()
      .getStructTopic("Camera1_EstimatedPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> cameraEstimatedPosePublisher2 = NetworkTableInstance.getDefault()
      .getStructTopic("Camera2_EstimatedPose", Pose2d.struct).publish();
  List<StructPublisher<Pose2d>> cameraEstimatedPosesPublisher = Arrays.asList(cameraEstimatedPosePublisher1, cameraEstimatedPosePublisher2);  

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  public static final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Vision vision = new Vision(Vision.camerasFromConfigs(VisionConstants.CONFIGS));

  /*
   * Named Commands Constants
   */

  public static final String ELEVATOR_TO_LEVEL_4_AUTO = "Elevator to Level4 Auto";
  private final String SCORE_LEVEL_4_CORAL = "Score Level4 Coral";
  private final String ELEVATOR_TO_LEVEL_4 = "Elevator To Level4";
  private final String ELEVATOR_TO_SELECTED_LEVEL = "Elevator To Selected Level";
  public static final String ELEVATOR_TO_HOME = "Elevator To Home";
  public static final String OUTPUT_CORAL = "Output Coral";
  private static final String NO_SELECTED_AUTO = "None";

  // region FeatureSwitches
  public static final boolean DEBUG_CONSOLE_LOGGING = true;
  public static final boolean AMBIGUITY_FILTER = true;
  public static final boolean LONG_DISTANCE_FILTER = true;
  public static final boolean RESET_CAMERA_RESULTS = false;
  public static final boolean VISION_ODOMETRY_ESTIMATION = true; // Enable vision and publish its estimated position
                                                                 // (doesn't update robot odometry)
  public static boolean VISION_ROBOT_ODOMETRY_UPDATE = true; // Enable vision odometry updates while driving. Doesn't
                                                             // work without VISION_ODOMETRY_ESTIMATION set to true.
  public static final boolean SIMULATE_VISION_FAILURES = false; // simulate dropped frames from the camera's 
  // endregion FeatureSwitches

  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    DriverStation.silenceJoystickConnectionWarning(true);
    registerCommands();
    SmartDashboard.putBoolean("Auto Mode Enable", false);
    autoChooser = AutoBuilder.buildAutoChooser("P2P_DS_Right_3Piece_ParallelIntake");
    TestAuto.configureAutos(autoChooser, drivetrain);
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
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

    driver.rightBumper().toggleOnTrue(IntakeCommands.intakeCoral(intake));
    // driver.rightBumper().toggleOnTrue(IntakeCommands.toggleIntakeCoral(intake));
    driver.leftBumper()
        .onTrue(new SequentialCommandGroup(IntakeCommands.expelCoral(intake), new ArmPosition(elevator, () -> ArmLevel.Travel)));
    driver.a().onTrue(new SequentialCommandGroup(new ArmPosition(elevator, () -> ArmLevel.Climb)));

    /* B Button: Auto Score  */
    driver.b().whileTrue(
      drivetrain.runAutoScore(() -> reefSelecter.getRobotPositionForSelectedCoral(), reefSelecter::getLevel, intake, elevator)
    ).onFalse(new MoveCoral(elevator, () -> ElevationLevel.Home, intake));

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
    // operator.b().onTrue(new GrabAlgae(elevator, intake, () -> {
    //   return highAlgae;
    // }));

    operator.y().onTrue(new MoveCoral(elevator, reefSelecter::getLevel, intake));
    operator.a().onTrue(new MoveCoral(elevator, () -> ElevationLevel.Home, intake));
    operator.x().whileTrue(IntakeCommands.runVoltage(intake, 4.0));

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
      },
      () -> operator.back().getAsBoolean(),
      () -> operator.start().getAsBoolean()
    );

    climbCommand.setName("Climb Command");
    SmartDashboard.putData(climbCommand);

    operator.start().and(operator.back()).toggleOnTrue(climbCommand); // left trigger connects the intake, right trigger releases

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

  void registerCommands() {

    NamedCommands.registerCommand(ELEVATOR_TO_LEVEL_4,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake)));
    NamedCommands.registerCommand(ELEVATOR_TO_SELECTED_LEVEL,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> reefSelecter.getLevel(), intake)));

    NamedCommands.registerCommand(ELEVATOR_TO_HOME,
        new MoveCoral(elevator, () -> ElevationLevel.Home, intake));

    NamedCommands.registerCommand(OUTPUT_CORAL,
        new ParallelRaceGroup(
            IntakeCommands.expelCoral(intake),
            new ArmPosition(elevator, () -> ArmLevel.Travel).beforeStarting(Commands.waitSeconds(0.25))));

    NamedCommands.registerCommand(SCORE_LEVEL_4_CORAL,
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake),
            IntakeCommands.expelCoral(intake), new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveCoral(elevator, () -> ElevationLevel.Home, intake)));
    NamedCommands.registerCommand("Score Level3 Coral",
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_3, intake),
            IntakeCommands.expelCoral(intake), new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveCoral(elevator, () -> ElevationLevel.Home, intake)));
    NamedCommands.registerCommand("Score Level2 Coral",
        new SequentialCommandGroup(new MoveCoral(elevator, () -> ElevationLevel.Level_2, intake),
            IntakeCommands.expelCoral(intake), new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveCoral(elevator, () -> ElevationLevel.Home, intake)));

    NamedCommands.registerCommand(ELEVATOR_TO_LEVEL_4_AUTO,
        Commands.sequence(Commands.waitUntil(intake::hasCoral), new MoveCoral(elevator, () -> ElevationLevel.Level_4, intake)).unless(() -> !intake.hasCoral()));

    NamedCommands.registerCommand(IntakeCommands.INTAKE_CORAL, IntakeCommands.intakeCoral(intake));

    PidToPoseCommands.registerCommands(drivetrain);
  }

  public void correctOdometry() {
    if (SIMULATE_VISION_FAILURES){
      int percentageFramesToDrop = 80;
      Random rnd = new Random();

      if(rnd.nextInt(100) < percentageFramesToDrop){
        return;
      }
    }

    List<VisionSample> visionSamples = vision.flushSamples();
    vision.updateSpeeds(drivetrain.getState().Speeds);
    // System.out.println("vision sample count: " + visionSamples.size());
    for (var sample : visionSamples) {
      double thetaStddev = sample.weight() > 0.9 ? 10.0 : 99999.0;
      drivetrain.addVisionMeasurement(
        sample.pose(),
        sample.timestamp(),
        VecBuilder.fill(0.1 / sample.weight(), 0.1 / sample.weight(), thetaStddev)
      );
    }

    for (int i = 0; i < 2; i++){
      if (i+1 <= visionSamples.size()){
        cameraEstimatedPosesPublisher.get(i).set(visionSamples.get(i).pose());
      }
    }
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
