// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RobotDriveCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.Autos;
import frc.robot.commands.Climb;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Elavator;
import frc.robot.lib.ReefSelecter;
import java.util.Optional;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TurnToTarget;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Elavator.ElevationControl;

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
  private final Elavator elavator = new Elavator();
  private final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driver.b().onTrue( new InstantCommand( () -> {
      elavator.setLevel(reefSelecter.getLevel());
    }));
    driver.a().onTrue( new InstantCommand( () -> {
      elavator.setLevel(Elavator.Level.Home);
    }));

    //TODO: remove me I am just a test
    driver.x().onTrue(new TurnToTarget(this::getXSpeed, this::getYSpeed, new Pose2d(1, 1, new Rotation2d(90)), driveBase));
  

    operator.leftBumper()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setDirection(ReefSelecter.Direction.Left) ;
            } ));
     operator.rightBumper()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setDirection(ReefSelecter.Direction.Right) ;
            } ));       
    operator.a()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setLevel(Elavator.Level.Level_1) ;
            } ));
    operator.x()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setLevel(Elavator.Level.Level_2) ;
            } ));
    operator.b()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setLevel(Elavator.Level.Level_3) ;
            } ));   
    operator.y()
            .onTrue( new InstantCommand( () -> { 
              reefSelecter.setLevel(Elavator.Level.Level_4) ;
    } ));
    operator.back().onTrue( new InstantCommand( driveBase::resetGyro ) {
        public boolean runsWhenDisabled() {
          return true;
        }    
      });    
    
    Command climbCommand = new Climb(climber, () -> {
      return operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
    });
    climbCommand.setName("Climb Command");
    climber.setDefaultCommand(climbCommand);
    SmartDashboard.putData(climbCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
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
}
