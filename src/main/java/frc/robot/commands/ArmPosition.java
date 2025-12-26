// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.lib.FinneyCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.lib.FinneyLogger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmPosition extends FinneyCommand {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  /** Creates a new CoralOutput. */
  private Elevator elevator;
  private Supplier<ArmLevel> desiredLevel;
  public ArmPosition(Elevator elevator, Supplier<ArmLevel> level) {
    this.elevator = elevator;
    this.desiredLevel = level;
    addRequirements(elevator);

    this.setName("ArmPosition");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    elevator.setArmlevel(desiredLevel.get());

    fLogger.log("Initializing ArmPosition to level: " + desiredLevel.get().toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevator.stopArm();

    fLogger.log("Ending ArmPosition at level %s with position %.1f, interrupted: %s", desiredLevel.get().toString(), elevator.getArmPosition(), interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isArmAtLevel(desiredLevel.get());
  }
}
