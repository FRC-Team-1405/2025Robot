// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  private Climber climber;
  private DoubleSupplier distance;
  private BooleanSupplier open;
  private BooleanSupplier close;

  private Servo myServo;
  /** Creates a new Climb. */
  public Climb(Climber climber, DoubleSupplier distance, BooleanSupplier open, BooleanSupplier close) {
    this.climber = climber;
    this.distance = distance;
    this.open = open;
    this.close = close;
    myServo = new Servo(9);
    myServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myServo.set(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  private static final double Deadband = 0.1;
  @Override
  public void execute() {
    double distance = MathUtil.applyDeadband(this.distance.getAsDouble(), Deadband);
    climber.move(distance);

    
    // if (open.getAsBoolean() && !close.getAsBoolean()) {
    //   myServo.setPulseTimeMicroseconds(1000);
    // } else if (!open.getAsBoolean() && close.getAsBoolean()) {
    //   myServo.setPulseTimeMicroseconds(2000);
    // } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      climber.stop();
    myServo.set(1.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
