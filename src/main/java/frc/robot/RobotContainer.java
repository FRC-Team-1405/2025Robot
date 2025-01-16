// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveDrive driveBase = new SwerveDrive(10, 2*Math.PI, "geared upright",  Constants.kinematics);
  // The robot's subsystems and commands are defined here...

  private Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private static final SendableChooser<String> autos = new SendableChooser<>();
  private SendableChooser<String> selectedAuto = new SendableChooser<String>();
  private static final String NO_SELECTED_AUTO = "None";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configurePathPlanner();
    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, this::getYSpeed, this::getRotationSpeed, this::getSlideValue, driveBase));
  }

  public void disabledInit() {
    driveBase.brakeMode(false);
  }
  
  public void autonomousInit() {
    driveBase.brakeMode(true);
  }

  public void teleopInit() {
    alliance = DriverStation.getAlliance();
    driveBase.brakeMode(false);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    operator.back().onTrue( new InstantCommand( driveBase::resetGyro ) {
        public boolean runsWhenDisabled() {
          return true;
        }    
      });

  }

  public Command getAutonomousCommand() {
    String autoName = selectedAuto.getSelected();
    if (autoName == NO_SELECTED_AUTO)
      return new PrintCommand("No Auto Selected");
    else 
      return new PathPlannerAuto(autoName);
  }

  double getXSpeed() { 
    double speedMultiplication = 0.6;
    speedMultiplication += (driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()) * 0.4;

    double finalX;
    if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY();
    
    return -finalX * speedMultiplication;
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
    
    return -finalY * speedMultiplication; 
  } 
  
  public double getRotationSpeed() { 
    double finalRotation =  driver.getRightX();

    if (Math.abs(finalRotation) < 0.15)
        finalRotation = 0.0;
    
    return finalRotation;
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
    var autoNames = AutoBuilder.getAllAutoNames();
    selectedAuto.addOption(NO_SELECTED_AUTO, NO_SELECTED_AUTO);
    autoNames.forEach((name) -> {
      selectedAuto.addOption(name, name);
    });
    SmartDashboard.putData("Selected Auto", selectedAuto);
  }
}
