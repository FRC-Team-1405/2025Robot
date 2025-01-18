// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Elavator;
import frc.robot.lib.ReefSelecter;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elavator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Elavator elavator = new Elavator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorPort);

  private final ReefSelecter reefSelecter = new ReefSelecter();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
      elavator.setlevel(reefSelecter.getLevel());
    }));
    driver.a().onTrue( new InstantCommand( () -> {
      elavator.setlevel(Elavator.Level.Home);
    }));
  

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
}
