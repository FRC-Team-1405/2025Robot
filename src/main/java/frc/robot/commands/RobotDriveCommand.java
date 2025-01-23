// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;

public class RobotDriveCommand extends SwerveDriveCommand {
  /** Creates a new RobotDriveCommand. */
  public RobotDriveCommand( DoubleSupplier getXSpeed, 
                            DoubleSupplier getYSpeed, 
                            DoubleSupplier getRotationSpeed, 
                            DoubleSupplier getSlide,
                            SwerveDrive driveBase) {
    super(getXSpeed, getYSpeed, getRotationSpeed, getSlide, driveBase);
    fieldOriented = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    SmartDashboard.putBoolean("Drive by Field", fieldOriented);
  }

}
