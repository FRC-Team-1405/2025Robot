// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.Elavator.ArmLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmPosition extends Command {
  /** Creates a new CoralOutput. */
  private Elavator elavator;
  private Supplier<ArmLevel> desiredLevel;
  public ArmPosition(Elavator elavator, Supplier<ArmLevel> level) {
    this.elavator = elavator;
    this.desiredLevel = level;
    addRequirements(elavator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elavator.setArmlevel(desiredLevel.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elavator.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elavator.isArmAtLevel(desiredLevel.get());
  }
}
