// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  private Elevator elavator;
  private Supplier<Elevator.ElevationLevel> level;
  /** Creates a new PlaceCoral. */
  public MoveElevator( Elevator elavator, Supplier<Elevator.ElevationLevel> level) {
    this.elavator = elavator;
    this.level = level;

    addRequirements(elavator);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Tell the motor where to go
  @Override
  public void initialize() {
      elavator.setLevel(level.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   }
  // Stops if interrupted
  @Override
  public void end(boolean interrupted) {
      if (interrupted) {
        elavator.stopElevator();
      }
  }

  // Checks motor positoin and stops the elevator
  @Override
  public boolean isFinished() {
    return elavator.isAtPosition();
  }
}
