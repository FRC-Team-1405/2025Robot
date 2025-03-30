// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveCoral extends SequentialCommandGroup {

  private Supplier<Elevator.ElevationLevel> level;

  public MoveCoral(Elevator elavator, Supplier<Elevator.ElevationLevel> level, Intake intake) {
    this.level = level;
    
    addRequirements(elavator);
    addCommands( intake.runOnce( intake::stop ));
    addCommands( new ArmPosition(elavator, () -> ArmLevel.Travel));
    addCommands( new MoveElevator(elavator, level));
    addCommands( new ArmPosition(elavator, this::armLevel) );
  }

  private ArmLevel armLevel()
  {
    ArmLevel value = switch((level.get())){
      case  Home -> ArmLevel.Home;
      case  Level_1 -> ArmLevel.Low_Score;
      case  Level_2 -> ArmLevel.Middle_Score;
      case  Level_3 -> ArmLevel.Middle_Score;
      case  Level_4 -> ArmLevel.High_Score;
      default -> ArmLevel.Home;
  };
    return value;
  }
}
