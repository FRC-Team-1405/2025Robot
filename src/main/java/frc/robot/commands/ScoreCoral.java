// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elavator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup {
  public ScoreCoral(Elavator elavator, Supplier<Elavator.Level> level) {
    addRequirements(elavator);
    addCommands( new PlaceCoral(elavator, level));
    addCommands( new PrintCommand("At Level " + level.get()));
    addCommands( new PlaceCoral(elavator, () -> { return Elavator.Level.Home; }));
  }
}
