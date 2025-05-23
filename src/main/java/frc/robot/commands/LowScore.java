// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowScore extends SequentialCommandGroup {
  /** Creates a new LowScore. */
  public LowScore(Elevator elavator,Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elavator, intake);
    addCommands(new ArmPosition(elavator, () -> ArmLevel.Travel));
    addCommands( new MoveElevator(elavator, () -> ElevationLevel.Inverted_Low));
    addCommands( new ArmPosition(elavator, () -> ArmLevel.Inverted_Low));
    addCommands( intake.runOnce(intake::slowScore)
                        .andThen(Commands.waitSeconds(2.0))
                        .andThen(intake.runOnce(intake::stop)));
    addCommands(new ArmPosition(elavator, () -> ArmLevel.Travel));
    addCommands(new MoveElevator(elavator, () -> ElevationLevel.Home));
    addCommands(new ArmPosition(elavator, () -> ArmLevel.Home));
   };
  
}
