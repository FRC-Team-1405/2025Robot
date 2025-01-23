// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends Command {
  private SwerveDrive driveBase;
  private DoubleSupplier getXSpeed, getYSpeed, getRotationSpeed; 
  private DoubleSupplier getSlide;
  protected boolean fieldOriented = true;

  public SwerveDriveCommand(DoubleSupplier getXSpeed, 
                            DoubleSupplier getYSpeed, 
                            DoubleSupplier getRotationSpeed, 
                            DoubleSupplier getSlide,
                            SwerveDrive driveBase) {
    this.getXSpeed = getXSpeed; 
    this.getYSpeed = getYSpeed; 
    this.getRotationSpeed = getRotationSpeed; 
    this.getSlide = getSlide;
    this.driveBase = driveBase; 
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  
  public void initialize() {
    SmartDashboard.putBoolean("Drive by Field", fieldOriented);
  }

  @Override
  public void execute() {
    double slideValue = getSlide.getAsDouble();

    if (slideValue == 0.0) {
      driveBase.drive(getXSpeed.getAsDouble(), getYSpeed.getAsDouble(), getRotationSpeed.getAsDouble(), fieldOriented);
    } else {
      driveBase.drive(slideValue, getYSpeed.getAsDouble(), getRotationSpeed.getAsDouble(), fieldOriented);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
  }
}