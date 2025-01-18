// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.LimelightHelpers;
import frc.robot.sensors.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionDrive extends Command {
  private SwerveDrive driveBase;
  private static double ROTATION_SPEED = 0.15;

  private enum LimelightData{
    X(0), 
    Y(1), 
    Z(2), 
    Pitch(3), 
    Yaw(4), 
    Roll(5);

    public final int index;
    private LimelightData(int index) {
      this.index = index;
    }
  }

  public VisionDrive(SwerveDrive driveBase) {
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTA("") > 0) {
      double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace("");
      SmartDashboard.putNumber("Vision/X",      targetPose[LimelightData.X.index]);
      SmartDashboard.putNumber("Vision/Y",      targetPose[LimelightData.Y.index]);
      SmartDashboard.putNumber("Vision/Z",      targetPose[LimelightData.Z.index]);
      SmartDashboard.putNumber("Vision/Pitch",  targetPose[LimelightData.Pitch.index]);
      SmartDashboard.putNumber("Vision/Yaw",    targetPose[LimelightData.Yaw.index]);
      SmartDashboard.putNumber("Vision/Roll",   targetPose[LimelightData.Roll.index]);
      // driveBase.drive(0, 0, targetPose[LimelightData.X.index] * ROTATION_SPEED, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
