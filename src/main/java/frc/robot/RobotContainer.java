// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Climb;
import frc.robot.commands.CoralInput;
import frc.robot.commands.CoralOutput;
import frc.robot.commands.CoralPositionCalculator;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.LowScore;
import frc.robot.commands.MoveCoral;
import frc.robot.lib.ReefSelecter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));

  private Optional<Alliance> alliance = DriverStation.getAlliance();

  private final Elevator elevator = new Elevator();
  private final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);

  /*
   * Named Commands Constants
   */

  private final String SCORE_LEVEL_4_CORAL = "Score Level4 Coral";
  private final String ELEVATOR_TO_LEVEL_4 = "Elevator To Level4";
  private final String ELEVATOR_TO_SELECTED_LEVEL = "Elevator To Selected Level";
  private final String ELEVATOR_TO_HOME = "Elevator To Home";
  private final String OUTPUT_CORAL = "Output Coral";

  //region FeatureSwitches

  public static final boolean AMBIGUITY_FILTER = true;
  public static final boolean LONG_DISTANCE_FILTER = true;
  public static final boolean RESET_CAMERA_RESULTS = false;
  public static final boolean PARALLEL_AUTO_ALIGN = true;
  public static final boolean CALCULATE_CORAL_ROBOT_POSITIONS = false;
  public static final boolean VISUALIZE_REEF_SELECTER_POSITION = false;
  public static final boolean AUTO_ALIGN_USE_SELECTED_ELEVATOR_LEVEL = true;
  public static final boolean VISION_ODOMETRY_ESTIMATION = true;  // Enable vision and publish its estimated position (doesn't update robot odometry)
  public static boolean VISION_ROBOT_ODOMETRY_UPDATE = false;  // Enable vision odometry updates while driving. Doesn't work without VISION_ODOMETRY_ESTIMATION set to true.
  
  /*
   * elastic dashboard button that moves robot to position.
   * temporarily enables VISION_ROBOT_ODOMETRY_UPDATE, via the Vision.visionUpdateRobotOdometry override, so that the camera can be used to align the robot.
   * this is used for drive odometry profiling.
   * move the robot to the auto's start position, then with VISION_ROBOT_ODOMETRY_UPDATE=false run the auto and take the error between the drive odometry and the vision odometry
   */
  public static final boolean MOVE_ROBOT_TO_POSITION = true;                  // elastic dashboard button that moves robot to position, 
  
  //endregion FeatureSwitches

  /*
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
      this::getXSpeed,
      this::getYSpeed)
      .withControllerRotationAxis(this::getRotationSpeed)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  private static final SendableChooser<String> autos = new SendableChooser<>();
  private SendableChooser<String> selectedAuto = new SendableChooser<String>();
  private static final String NO_SELECTED_AUTO = "None";

  private static final String SimpleDrive = "Simple Drive";
  private static final String InvertedScore = "Inverted Score";
  private static final String DriveToReef = "Drive To Reef";
  private static final String DriveAndScoreLow = "Drive And Score Low";
  private static final String DriveAndScoreHigh = "Drive And Score High";
  private static final String DiagonalScore = "Diagonal Score";

  private CoralPositionCalculator coralPositionCalculator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configurePathPlanner();
    configureBindings();
    configureShuffboardCommands();

    if (CALCULATE_CORAL_ROBOT_POSITIONS) {
      coralPositionCalculator = new CoralPositionCalculator(() -> driveBase.getPose());
    }

    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveAngularVelocity));
  }

  public void disabledInit() {
    climber.stop();
  }

  public void autonomousInit() {
  }

  public void teleopInit() {
    alliance = DriverStation.getAlliance();
    reefSelecter.updateSelectedReefPositionVisualizer();
  }

  private void configureShuffboardCommands() {
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

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private boolean highAlgae = true;

  private void configureBindings() {
    driver.rightBumper().toggleOnTrue(new CoralInput(intake));
    driver.leftBumper()
        .onTrue(new SequentialCommandGroup(new CoralOutput(intake), new ArmPosition(elevator, () -> ArmLevel.Travel)));
    driver.a().onTrue(new SequentialCommandGroup(new ArmPosition(elevator, () -> ArmLevel.Climb)));
    driver.back().onTrue((Commands.runOnce(driveBase::zeroGyroWithAlliance)).ignoringDisable(true));

    

    if (PARALLEL_AUTO_ALIGN) {

      // Driver presses AND HOLDS B to activate auto align. auto align will move to the scoring position while raising the elevator.
      // It will not score the coral, the operator will need to output the coral.
      // when the driver lets go the auto align will stop. the elevator will not move until operator moves it.
      if (AUTO_ALIGN_USE_SELECTED_ELEVATOR_LEVEL) {
        driver.b()
        .whileTrue(driveBase
            .driveToPose(
              () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition())
            ).alongWith(NamedCommands.getCommand(ELEVATOR_TO_SELECTED_LEVEL)));
      } else {
        driver.b()
        .whileTrue(driveBase
            .driveToPose(
              () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition())
            ).alongWith(NamedCommands.getCommand(ELEVATOR_TO_LEVEL_4)));
      }
      
    } else {

      // Driver presses AND HOLDS B to activate auto align. auto align will move to the scoring position, raise the elevator and score the coral in sequence.
      // when the driver lets go the auto align will stop. the elevator will not move until operator moves it.
      if (AUTO_ALIGN_USE_SELECTED_ELEVATOR_LEVEL) {
        driver.b()
          .whileTrue(driveBase
              .driveToPose(
                () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition())
              )
              .andThen(
                  NamedCommands.getCommand(ELEVATOR_TO_SELECTED_LEVEL)));
      } else {
        driver.b()
        .whileTrue(driveBase
            .driveToPose(
              () -> reefSelecter.getRobotPositionForCoral(reefSelecter.getCoralPosition())
            )
            .andThen(
                NamedCommands.getCommand(ELEVATOR_TO_LEVEL_4)));
      }
    }

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

    if(RobotBase.isSimulation()){
      driver.x().onTrue(Commands.runOnce(driveBase::addFakeVisionReading));
    }
  }

  public Command getAutonomousCommand() {
    if (DriverStation.isFMSAttached() || SmartDashboard.getBoolean("Auto Mode Enable", false)) {
      SmartDashboard.putBoolean("Auto Mode Enable", false);
      String autoName = selectedAuto.getSelected();
      if (autoName == NO_SELECTED_AUTO) {
        return Commands.none();
      } else {
        return new PathPlannerAuto(autoName);
      }
    } else {
      return Commands.print("Auto Disabled");
    }
  }

  // TODO use driveAngularVelocity's scaleTranslation method and remove this
  // method.
  public double getXSpeed() {
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);

    return driver.getLeftY() * speedMultiplication * -1;
  }

  public double getYSpeed() {
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);

    int pov = driver.getHID().getPOV();

    double finalY;
    if (pov == 270 || pov == 315 || pov == 225)
      finalY = -.5;
    else if (pov == 90 || pov == 45 || pov == 135)
      finalY = 0.5;
    else
      finalY = driver.getLeftX();

    return finalY * speedMultiplication * -1;
  }

  public double getRotationSpeed() {
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);

    return driver.getRightX() * speedMultiplication * -1;
  }

  public void setMotorBrake(boolean brake) {
    driveBase.setMotorBrake(brake);
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
            new ArmPosition(elevator, () -> ArmLevel.Travel).beforeStarting(Commands.waitSeconds(0.25))
        ));

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

    var autoNames = AutoBuilder.getAllAutoNames();
    selectedAuto.addOption(NO_SELECTED_AUTO, NO_SELECTED_AUTO);
    selectedAuto.addOption(SimpleDrive, SimpleDrive);
    selectedAuto.addOption(DriveToReef, DriveToReef);
    selectedAuto.addOption(InvertedScore, InvertedScore);
    selectedAuto.addOption(DriveAndScoreLow, DriveAndScoreLow);
    selectedAuto.addOption(DriveAndScoreHigh, DriveAndScoreHigh);
    selectedAuto.addOption(DiagonalScore, DiagonalScore);
    autoNames.forEach((name) -> {
      selectedAuto.addOption(name, name);
    });
    SmartDashboard.putData("Selected Auto", selectedAuto);
  }
}
