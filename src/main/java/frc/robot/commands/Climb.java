// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  private Climber climber;
  private DoubleSupplier distance;
  private boolean holdPosition = false;
  /** Creates a new Climb. */
  public Climb(Climber climber, DoubleSupplier distance) {
    this.climber = climber;
    this.distance = distance;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.holdPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  private static final double Deadband = 0.1;
  @Override
  public void execute() {
    double distance = MathUtil.applyDeadband(this.distance.getAsDouble(), Deadband);
    if (MathUtil.isNear(0.0, distance, Deadband)){
      if (!holdPosition){
        climber.holdPosition();
      }
      holdPosition = true;
    } else {
        climber.move(distance);
        holdPosition = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      climber.stop();
    else
      climber.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
