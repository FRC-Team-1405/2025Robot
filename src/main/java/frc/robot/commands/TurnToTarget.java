// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToTarget extends Command {
  private static double z_P = 0.5 ;
  private static double z_I = 0.0 ;
  private static double z_D = 0.0 ;
  private Translation2d target;
  private SwerveDrive swerve;
  private ProfiledPIDController zController;
  private DoubleSupplier getXSpeed;
  private DoubleSupplier getYSpeed;
  
  /** Creates a new TurnToTarget. */
  public TurnToTarget(DoubleSupplier getXSpeed, DoubleSupplier getYSpeed, Translation2d target, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.swerve = swerve;
    this.getXSpeed = getXSpeed;
    this.getYSpeed = getYSpeed;
    configPIDs(swerve);
    loadConfigs();
    addRequirements(swerve);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    checkSmartdashboard();
    zController.reset(swerve.getPose().getRotation().getRadians());    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goal = getTargetAngle();
    double swerveRot = swerve.getPose().getRotation().getRadians();
    double rotationSpeed = MathUtil.clamp(zController.calculate( swerveRot, goal ), -1, 1);

    // This actualy takes into account the tolerance that we set earlier because the calculate() doesn't do it internally
    // Hopefully this should help with the death spins maybe
    if(!zController.atGoal()) {
      swerve.drive(getXSpeed.getAsDouble(), getYSpeed.getAsDouble(), -rotationSpeed, true);
    } else {
      swerve.drive(getXSpeed.getAsDouble(), getYSpeed.getAsDouble(), 0, true);
    }

    SmartDashboard.putNumber("TurnToTarget/PIDError", zController.getPositionError());
    SmartDashboard.putNumber("TurnToTarget/Setpoint", zController.getSetpoint().position);
    SmartDashboard.putNumber("TurnToTarget/Target Angle", goal);
    SmartDashboard.putNumber("TurnToTarget/Current Angle", swerveRot);
    SmartDashboard.putNumber("TurnToTarget/Rot Speed", rotationSpeed);
  }

  private void checkSmartdashboard() {
    z_P = Preferences.getDouble("TurnToTarget/Z/P", z_P);
    z_I = Preferences.getDouble("TurnToTarget/Z/I", z_I);
    z_D = Preferences.getDouble("TurnToTarget/Z/D", z_D);
    zController.setPID(z_P, z_I, z_D);

    target = new Translation2d(Preferences.getDouble("TurnToTarget/Target/X", target.getY()), 
                               Preferences.getDouble("TurnToTarget/Target/Y", target.getY()));
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
    // double swerveRot = swervePose.getRotation().getRadians();
    double targetX = target.getX();
    double targetY = target.getY();

    // Counter-Clockwise positive
    double angle = Math.atan2((targetY - swerveY), (targetX - swerveX));
    return angle;
  }

  private void configPIDs(SwerveDrive swerve){
    zController = new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(4,4));
    zController.enableContinuousInput(-Math.PI, Math.PI); 
    zController.setTolerance((Math.PI*2)/100.0);
    zController.reset(swerve.getPose().getRotation().getRadians());
  } 

  private void loadConfigs(){
    Preferences.initDouble("TurnToTarget/Z/P", z_P);
    Preferences.initDouble("TurnToTarget/Z/I", z_I);
    Preferences.initDouble("TurnToTarget/Z/D", z_D);

    Preferences.initDouble("TurnToTarget/Target/X", target.getX());
    Preferences.initDouble("TurnToTarget/Target/Y", target.getY());

    checkSmartdashboard();
  }
}
