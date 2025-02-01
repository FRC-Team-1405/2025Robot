// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToTarget extends Command {
  private static double z_P = 10.0 ;
  private static double z_I = 0.0 ;
  private static double z_D = 0.0 ;
  private Pose2d target;
  private SwerveDrive swerve;
  private ProfiledPIDController zController;
  private DoubleSupplier getXSpeed;
  private DoubleSupplier getYSpeed;
  
  /** Creates a new TurnToTarget. */
  public TurnToTarget(DoubleSupplier getXSpeed, DoubleSupplier getYSpeed, Pose2d target, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    //  loadConfigs();
     configPIDs(swerve);
      this.target = target;
      this.swerve = swerve;
      this.getXSpeed = getXSpeed;
      this.getYSpeed = getYSpeed;
      addRequirements(swerve);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = getTargetAngle();
    double swerveRot = swerve.getPose().getRotation().getRadians();
    double goal = Math.IEEEremainder(angle - swerveRot, Math.PI);
    zController.setGoal(angle);
    double rotationSpeed = zController.calculate( swerveRot );
    SmartDashboard.putNumber("TurnToTarget/PIDError", zController.getPositionError());
    SmartDashboard.putNumber("TurnToTarget/Target Angle", angle);
    SmartDashboard.putNumber("TurnToTarget/Current Angle", swerveRot);
    SmartDashboard.putNumber("TurnToTarget/Rot Speed", rotationSpeed);
    SmartDashboard.putNumber("TurnToTarget/Goal", goal);

    swerve.drive(getXSpeed.getAsDouble(), getYSpeed.getAsDouble(), -rotationSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getTargetAngle() {
    Pose2d swervePose = swerve.getPose();
    double swerveX = swervePose.getTranslation().getX();
    double swerveY = swervePose.getTranslation().getY();
    double swerveRot = swervePose.getRotation().getRadians();
    double targetX = target.getTranslation().getX();
    double targetY = target.getTranslation().getY();
    //double targetRot = target.getRotation().getRadians();

    // Counter-Clockwise positive
    double angle = Math.atan((targetX - swerveX)/(targetY - swerveY));
    return angle;
  }

  private void configPIDs(SwerveDrive swerve){
    zController =  new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(1,1));
    zController.enableContinuousInput(-Math.PI, Math.PI); 
    zController.setTolerance((Math.PI*2)/10.0);
  } 

  // private static void loadConfigs(){
  //   Preferences.initDouble("TurnToTarget/Z/P", z_P);
  //   Preferences.initDouble("TurnToTarget/Z/I", z_I);
  //   Preferences.initDouble("TurnToTarget/Z/D", z_D);
  //   z_P = Preferences.getDouble("TurnToTarget/Z/P", z_P);
  //   z_I = Preferences.getDouble("TurnToTarget/Z/I", z_I);
  //   z_D = Preferences.getDouble("TurnToTarget/Z/D", z_D);
  // }

}
