// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.lib.ReefSelecter;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elavator;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.RobotDriveCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Climb;
import frc.robot.commands.CoralInput;
import frc.robot.commands.CoralOutput;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.subsystems.Elavator;
import frc.robot.lib.ReefSelecter;
import java.util.Optional;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Elavator.ArmLevel;
import frc.robot.subsystems.Elavator.ElevationControl;
import frc.robot.subsystems.Elavator.ElevationLevel;

/**
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveDrive driveBase = new SwerveDrive(10, 2*Math.PI, "geared upright",  Constants.kinematics);
  // The robot's subsystems and commands are defined here...

  private Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private final Elavator elavator = new Elavator();
  private final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);


  private static final SendableChooser<String> autos = new SendableChooser<>();
  private SendableChooser<String> selectedAuto = new SendableChooser<String>();
  private static final String NO_SELECTED_AUTO = "None";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configurePathPlanner();
    configureShuffboardCommands();
    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, this::getYSpeed, this::getRotationSpeed, this::getSlideValue, driveBase));
  }

  public void disabledInit() {
    driveBase.brakeMode(false);
    climber.stop();
  }
  
  public void autonomousInit() {
    driveBase.brakeMode(true);
  }

  public void teleopInit() {
    alliance = DriverStation.getAlliance();
    driveBase.brakeMode(false);
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
    
    SmartDashboard.putBoolean("Auto Mode Enable", false);
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
  private void configureBindings() {
    driver.y().onTrue( new MoveCoral(elavator, reefSelecter::getLevel) );
    driver.a().onTrue( new MoveCoral(elavator, () -> ElevationLevel.Home));
    driver.x().toggleOnTrue( new CoralInput(intake) );

    driver.rightBumper().onTrue( new CoralInput(intake) );
    driver.leftBumper().onTrue( new SequentialCommandGroup( new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel) ) );
    driver.start().onTrue(new InstantCommand(() -> driveBase.resetGyro()));

    operator.y().onTrue( new MoveCoral(elavator, reefSelecter::getLevel) );
    operator.a().onTrue( new MoveCoral(elavator, () -> ElevationLevel.Home));
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
    operator.back().onTrue( new InstantCommand( driveBase::resetGyro ) {
        public boolean runsWhenDisabled() {
          return true;
        }    
      }); 
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

  }
  
  public Command getAutonomousCommand() {
    if(DriverStation.isFMSAttached() || SmartDashboard.getBoolean("Auto Mode Enable", false)){
      SmartDashboard.putBoolean("Auto Mode Enable", false);
      String autoName = selectedAuto.getSelected();
      if (autoName == NO_SELECTED_AUTO){
        return Commands.none();
      }else{ 
        return new PathPlannerAuto(autoName);
      }
    } else{
        return Commands.print("Auto Disabled");
    }
  }

  public double getXSpeed(){
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * 0.4;

    double finalX;
    if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY();
    
    return finalX * speedMultiplication;
  }

  public double getYSpeed() { 
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * 0.4;
    
    int pov = driver.getHID().getPOV();

    double finalY;
    if ( pov == 270 || pov == 315 || pov == 225)
      finalY = -.5;
    else if(pov == 90 || pov == 45 || pov == 135)
      finalY = 0.5;
    else if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX();
    
    return finalY * speedMultiplication; 
  } 
  
  public double getRotationSpeed() { 
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * 0.4;

    double finalRotation =  -driver.getRightX();

    if (Math.abs(finalRotation) < 0.15)
        finalRotation = 0.0;
    
    return finalRotation * speedMultiplication;
  }
  
  public double getSlideValue() {
    int pov = driver.getHID().getPOV();
    if (pov == 45 || pov == 90 || pov == 135) {
      return 0.4 ;
    } else if (pov == 225 || pov == 270 || pov == 315) {
      return -0.4 ;
    }

    return 0.0;
  }

  void configurePathPlanner() {
    NamedCommands.registerCommand("Score Level4 Coral", 
                  new SequentialCommandGroup( new MoveCoral(elavator, () -> ElevationLevel.Level_4), 
                  new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel), 
                  new MoveCoral(elavator, () -> ElevationLevel.Home)));
    NamedCommands.registerCommand("Score Level3 Coral", 
                  new SequentialCommandGroup( new MoveCoral(elavator, () -> ElevationLevel.Level_3), 
                  new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel), 
                  new MoveCoral(elavator, () -> ElevationLevel.Home)));
    NamedCommands.registerCommand("Score Level2 Coral", 
                  new SequentialCommandGroup( new MoveCoral(elavator, () -> ElevationLevel.Level_2), 
                  new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel), 
                  new MoveCoral(elavator, () -> ElevationLevel.Home)));
    NamedCommands.registerCommand("Intake Coral", new CoralInput(intake));

    var autoNames = AutoBuilder.getAllAutoNames();
    selectedAuto.addOption(NO_SELECTED_AUTO, NO_SELECTED_AUTO);
    autoNames.forEach((name) -> {
      selectedAuto.addOption(name, name);
    });
    SmartDashboard.putData("Selected Auto", selectedAuto);
  }
}
