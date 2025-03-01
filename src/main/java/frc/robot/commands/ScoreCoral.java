// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.Elavator.ArmLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup {
  public ScoreCoral(Elavator elavator, Supplier<Elavator.ElevationLevel> level) {
    ArmLevel armLevel = switch((level.get())){
        case  Home -> ArmLevel.Home;
        case  Level_1 -> ArmLevel.Low_Score;
        case  Level_2 -> ArmLevel.Middle_Score;
        case  Level_3 -> ArmLevel.Middle_Score;
        case  Level_4 -> ArmLevel.High_Score;
        default -> ArmLevel.Home;
    };
    
    addRequirements(elavator);
    addCommands( new ArmPosition(elavator, ArmLevel.Travel));
    addCommands( new PlaceCoral(elavator, level));
    addCommands( new ArmPosition(elavator, armLevel) );
    addCommands( new PrintCommand("At Level " + level.get()));
    // addCommands( new ArmPosition(elavator, ArmLevel.Travel));
    // addCommands( new PlaceCoral(elavator, () -> { return Elavator.ElevationLevel.Home; }));
    // addCommands( new ArmPosition(elavator, ArmLevel.Home));
  }
}
