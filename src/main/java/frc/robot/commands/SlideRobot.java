// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SlideRobot extends Command {
  private SwerveDrive swerveDrive;
  private double start;
  private DoubleSupplier distance;
  /** Creates a new SlideRobot. */
  public SlideRobot(SwerveDrive swerveDrive, DoubleSupplier distance) {
    this.swerveDrive = swerveDrive;
    this.distance = distance;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = swerveDrive.getDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(distance.getAsDouble() < 0 ? 0.25 : 0.25, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swerveDrive.getDistance()-start) > distance.getAsDouble();
  }
}
