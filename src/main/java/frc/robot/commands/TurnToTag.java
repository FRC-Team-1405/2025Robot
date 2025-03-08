// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class TurnToTag extends TurnToTarget {
    private PhotonCamera camera;
    private double angle;

    public TurnToTag(PhotonCamera camera, DoubleSupplier getXSpeed, DoubleSupplier getYSpeed, SwerveDrive swerve) {
        super(getXSpeed, getYSpeed, new Translation2d(), swerve);
        this.camera = camera;
        angle = super.swerve.getPose().getRotation().getRadians();
    }

  @Override
  public double getTargetAngle() {
    Pose2d swervePose = super.swerve.getPose();
    double swerveRot = swervePose.getRotation().getRadians();
    
    var result = camera.getLatestResult();
    if(result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d targetSpace = target.getBestCameraToTarget();
        double targetYaw =  targetSpace.getRotation().getZ() + Math.PI;

        angle = swerveRot - targetYaw;
        SmartDashboard.putNumber("TurnToTarget/angle", Units.radiansToDegrees(angle));
        SmartDashboard.putNumber("TurnToTarget/actual Angle", Units.radiansToDegrees(swerveRot));
        SmartDashboard.putNumber("TurnToTarget/targetYaw", Units.radiansToDegrees(targetYaw));
    }

    // Counter-Clockwise positive
    return angle;
  }
}
