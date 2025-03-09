// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RobotDriveCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Climb;
import frc.robot.commands.CoralInput;
import frc.robot.commands.CoralOutput;
import frc.robot.commands.DropAlgae;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.MoveElevator;
import frc.robot.subsystems.Elavator;
import frc.robot.lib.ReefSelecter;
import java.util.Optional;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Elavator.ArmLevel;
import frc.robot.subsystems.Elavator.ElevationControl;
import frc.robot.subsystems.Elavator.ElevationLevel;

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
  
  private final Elavator elavator = new Elavator();
  private final ReefSelecter reefSelecter = new ReefSelecter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    
    Command zeroizeClimber = climber.runOnce(() -> { 
      climber.zeroize();
    });
    zeroizeClimber.setName("zeroize Climber");
    SmartDashboard.putData(zeroizeClimber);
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
    driver.rightBumper().toggleOnTrue( 
      Commands.either(
        new CoralInput(intake), 
        Commands.sequence(
          intake.runOnce( intake::intakeAlgae ),
          Commands.waitSeconds(1.0),
          intake.runOnce( intake::stop )
          ),
        () -> { return reefSelecter.getLevel() != ElevationLevel.Level_1; } )
    );

    driver.leftBumper().onTrue( new SequentialCommandGroup( new CoralOutput(intake), new ArmPosition(elavator, () -> ArmLevel.Travel) ) );

    driver.y().onTrue( new MoveCoral(elavator, reefSelecter::getLevel, intake) );
    driver.a().onTrue( new MoveCoral(elavator, () -> ElevationLevel.Home, intake));
    driver.back().onTrue( new InstantCommand( driveBase::resetGyro ) {
      public boolean runsWhenDisabled() {
        return true;
      }    
    }); 

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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  double getXSpeed() { 
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
}
