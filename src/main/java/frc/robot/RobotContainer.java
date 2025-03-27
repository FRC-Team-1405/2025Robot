// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import frc.robot.lib.ReefSelecter;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elavator;

import frc.robot.commands.ScoreCoral;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Climb;
import frc.robot.commands.CoralInput;
import frc.robot.commands.CoralOutput;
import frc.robot.commands.CoralSonar;
import frc.robot.commands.DropAlgae;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.LowScore;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Elavator.ArmLevel;
import com.pathplanner.lib.commands.PathPlannerAuto;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.Elavator.ArmLevel;
import frc.robot.subsystems.Elavator.ElevationControl;
import frc.robot.subsystems.Elavator.ElevationLevel;

/**
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem driveBase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve/falcon"));

  private Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private final Elavator elavator = new Elavator();
  private final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);

  /*
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configurePathPlanner();
    configureShuffboardCommands();
    configurePathPlanner();


    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveAngularVelocity));
  }

  public void disabledInit() {
    climber.stop();
  }
  
  public void autonomousInit() {
  }

  public void teleopInit() {
    alliance = DriverStation.getAlliance();
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

    Command lowScore = new LowScore(elavator, intake);
    lowScore.setName("Low Score");
    SmartDashboard.putData(lowScore);

    SmartDashboard.putNumber("Test/Short Drive Time", 0.5);
    SmartDashboard.putNumber("Test/Reef Drive", 2.0);
}

/**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private boolean highAlgae = true;
  private void configureBindings() {
    driver.rightBumper().toggleOnTrue( new CoralInput(intake) );
    driver.leftBumper().onTrue( new SequentialCommandGroup( new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel) ) );
    driver.a().onTrue(new SequentialCommandGroup(new ArmPosition(elavator, () -> ArmLevel.Climb)));
    driver.back().onTrue((Commands.runOnce(driveBase::zeroGyro)).ignoringDisable(true)); 

    SmartDashboard.putBoolean("Algae/High", highAlgae);
    SmartDashboard.putBoolean("Algae/Low", !highAlgae);
    operator.rightBumper().onTrue( Commands.runOnce( () -> { 
        highAlgae = true;
        SmartDashboard.putBoolean("Algae/High", highAlgae);
        SmartDashboard.putBoolean("Algae/Low", !highAlgae);
      } )) ;
    operator.leftBumper().onTrue( Commands.runOnce( () -> { 
        highAlgae = false; 
        SmartDashboard.putBoolean("Algae/High", highAlgae);
        SmartDashboard.putBoolean("Algae/Low", !highAlgae);
      } )) ;
    operator.b().onTrue( new GrabAlgae(elavator, intake, () -> {return highAlgae;}) );

    operator.y().onTrue( new MoveCoral(elavator, reefSelecter::getLevel, intake) );
    operator.a().onTrue( new MoveCoral(elavator, () -> ElevationLevel.Home, intake));
    operator.x().onTrue( new InstantCommand(() -> {
      intake.outtakeCoral();
    }));
    operator.x().onFalse( new InstantCommand(() -> {
      intake.stop();
    }));

    operator.povLeft()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setDirection(ReefSelecter.Direction.Left) ;
            } ));
     operator.povRight()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setDirection(ReefSelecter.Direction.Left) ;
            } ));       
      operator.povUp()
              .or(operator.povUpLeft())
              .or(operator.povUpRight())
              .onTrue( new InstantCommand( () -> {
                reefSelecter.levelUp();
              } ));

      operator.povDown()
              .or(operator.povDownLeft())
              .or(operator.povDownRight())
              .onTrue( new InstantCommand( () -> {
                reefSelecter.levelDown();
              }));


    Command climbCommand = new Climb(climber, () -> {
      return operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
    });
    climbCommand.setName("Climb Command");
    SmartDashboard.putData(climbCommand);

    operator.start().and(operator.back()).toggleOnTrue(climbCommand);


    Command coralSonar = new CoralSonar( driveBase::getPose, driver );
    Trigger hasCoral = new Trigger( intake::hasCoral ); 
    hasCoral.onTrue( coralSonar );
  }
  
  public Command getAutonomousCommand() {
    // return driveBase.getAutonomousCommand("CircleAuto");
    return driveBase.getAutonomousCommand("DriveStraight1m");
  }

  public double getXSpeed(){
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);

    return driver.getLeftY() * speedMultiplication;
  }

  public double getYSpeed() { 
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);
    
    int pov = driver.getHID().getPOV();

    double finalY;
    if ( pov == 270 || pov == 315 || pov == 225)
      finalY = -.5;
    else if(pov == 90 || pov == 45 || pov == 135)
      finalY = 0.5;
    else
      finalY = driver.getLeftX();
    
    return finalY * speedMultiplication; 
  } 

  public double getRotationSpeed() { 
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * (1 - speedMultiplication);

    return driver.getRightX() * speedMultiplication;
  }
  
  public void setMotorBrake(boolean brake) {
    driveBase.setMotorBrake(brake);
  }

  void configurePathPlanner() {

    NamedCommands.registerCommand("Score Level4 Coral", 
                  new SequentialCommandGroup( new MoveCoral(elavator, () -> ElevationLevel.Level_4, intake), 
                  new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel)));
    NamedCommands.registerCommand("Score Level3 Coral", 
                  new SequentialCommandGroup( new MoveCoral(elavator, () -> ElevationLevel.Level_3, intake), 
                  new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel), 
                  new MoveCoral(elavator, () -> ElevationLevel.Home, intake)));
    NamedCommands.registerCommand("Score Level2 Coral", 
                  new SequentialCommandGroup( new MoveCoral(elavator, () -> ElevationLevel.Level_2, intake), 
                  new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel), 
                  new MoveCoral(elavator, () -> ElevationLevel.Home, intake)));
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
