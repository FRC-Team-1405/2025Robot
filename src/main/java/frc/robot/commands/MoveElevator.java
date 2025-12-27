// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.lib.FinneyCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.lib.FinneyLogger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends FinneyCommand {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  private Elevator elevator;
  private Supplier<Elevator.ElevationLevel> level;
  /** Creates a new PlaceCoral. */
  public MoveElevator( Elevator elevator, Supplier<Elevator.ElevationLevel> level) {
    // Epilogue.bind(this); // Starts automatic logging
    DataLogManager.start(); // Optional: saves logs to disk

    this.elevator = elevator;
    this.level = level;

    addRequirements(elevator);

    this.setName("MoveElevator");
  }

  // Tell the motor where to go
  @Override
  public void initialize() {
    super.initialize();
    elevator.setLevel(level.get());
    fLogger.log("Initializing MoveElevator to level: " + level.get().toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   }
  // Stops if interrupted
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevator.stopElevator();
    fLogger.log("Ending MoveElevator at level %s with position %.1f, interrupted: %s", level.get().toString(), elevator.getElevatorPos(), interrupted);
  }

  // Checks motor positoin and stops the elevator
  @Override
  public boolean isFinished() {
    return elevator.isAtPosition(level.get().getposition());
  }
}
