// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPilot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.finneyrobotics.library.AllianceSymmetry;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPilotCommand extends Command {
  public final SwerveRequest.ApplyFieldSpeeds pidToPose_FieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);
  public final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  private static final APConstraints kConstraints = new APConstraints()
      .withAcceleration(5.0)
      .withJerk(2.0);

  private static final APProfile kProfile = new APProfile(kConstraints)
      .withErrorXY(Centimeters.of(2))
      .withErrorTheta(Degrees.of(0.5))
      .withBeelineRadius(Centimeters.of(8));

  public static final Autopilot kAutopilot = new Autopilot(kProfile);

    private APTarget m_target;
    private final Supplier<Pose2d> m_targetSupplier;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Optional<Rotation2d> m_entryAngle;
  private final boolean m_flipPoseForAlliance;
  private double startingDistanceFromTarget;
  private Pose2d startingPosition;

  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(
    2.0, 0.0, 0.0, // PID gains
    new TrapezoidProfile.Constraints(Math.PI, Math.PI) // max velocity and acceleration
);

  private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withHeadingPID(2, 0, 0); /* tune this for your robot! */


  public AutoPilotCommand(Supplier<Pose2d> target, CommandSwerveDrivetrain drivetrain) {
    this(target, drivetrain, Optional.empty());
  }

  public AutoPilotCommand(Supplier<Pose2d> target, CommandSwerveDrivetrain drivetrain, Optional<Rotation2d> entryAngle) {
    this(target, drivetrain, entryAngle, false);
  }

  public AutoPilotCommand(Supplier<Pose2d> target, CommandSwerveDrivetrain drivetrain, Optional<Rotation2d> entryAngle, boolean flipPoseForAlliance) {
    m_targetSupplier = target;
    m_drivetrain = drivetrain;
    m_entryAngle = entryAngle;
    m_flipPoseForAlliance = flipPoseForAlliance;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    /* no-op */
    if (m_flipPoseForAlliance && DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())) {
      // flip pose for red alliance
      m_target = new APTarget(AllianceSymmetry.flip(m_targetSupplier.get()));
    } else {
      m_target = new APTarget(m_targetSupplier.get());
    }
    
    startingDistanceFromTarget = getDistanceToTarget();
    startingPosition = m_drivetrain.getState().Pose;

    m_thetaController.reset(startingPosition.getRotation().getRadians());
  }

  @Override
  public void execute() {
    ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getState().Speeds;
    Pose2d pose = m_drivetrain.getState().Pose;
    // System.out.println(String.format("Robot Pose (x: %s, y: %s)", pose.getX(), pose.getY()));

    Optional<Rotation2d> correctedEntryAngle = m_entryAngle;

    if (correctedEntryAngle.isPresent()) {
        correctedEntryAngle = Optional.of( correctedEntryAngle.get().plus(m_target.getReference().getRotation()) ); // convert from input of robot relative entry angle to field relative entry angle
    }

    APResult out;
    if (m_entryAngle.isPresent()){
      out = kAutopilot.calculate(pose, robotRelativeSpeeds, m_target.withEntryAngle(correctedEntryAngle.get()));
    } else {
      out = kAutopilot.calculate(pose, robotRelativeSpeeds, m_target.withoutEntryAngle());
    }

    // APResult out = kAutopilot.calculate(pose, robotRelativeSpeeds, m_target);

    // m_drivetrain.setControl(m_request
    //     .withVelocityX(out.vx())
    //     .withVelocityY(out.vy())
    //     .withTargetDirection(out.targetAngle()));

    Rotation2d currentRotation = pose.getRotation();
    Rotation2d targetRotation = out.targetAngle(); // from Autopilot

    double thetaOutput = m_thetaController.calculate(
        currentRotation.getRadians(),
        targetRotation.getRadians()
    );

    ChassisSpeeds outRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        out.vx(),//.times(100),
        out.vy(),//.times(100),
        AngularVelocity.ofBaseUnits(thetaOutput, RadiansPerSecond),
        m_drivetrain.getState().Pose.getRotation()
    );


    //this worked
    // m_drivetrain.setControl(applyRobotSpeeds.withSpeeds(outRobotRelativeSpeeds));
    if(0.6 < getPercentageOfDistanceToTarget()){
      // drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(veloX.magnitude(), veloY.magnitude(), thetaOutput)));
      
      System.out.println("thetaOutput, " + thetaOutput + ", currentRotation: " + pose.getRotation().getDegrees() + ", targetRotation: " + m_target.getReference().getRotation().getDegrees());
      System.out.println("percentageToTarget: +60%");
      m_drivetrain.setControl(applyRobotSpeeds.withSpeeds(outRobotRelativeSpeeds));
    } else if(0.2 < getPercentageOfDistanceToTarget()){
      System.out.println("percentageToTarget: +20%");
      m_drivetrain.setControl(m_request
        .withVelocityX(out.vx())
        .withVelocityY(out.vy())
        .withTargetDirection(targetRotation));
    } else {
      System.out.println("percentageToTarget: +0%");
      // drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(veloX.magnitude(), veloY.magnitude(), 0)));
      m_drivetrain.setControl(m_request
        .withVelocityX(out.vx())
        .withVelocityY(out.vy()).withTargetDirection(startingPosition.getRotation()));
    }
  }

  @Override
  public boolean isFinished() {
    System.out.println(String.format("Angle Difference: %.1f, Target angle: %.1f, Current Angle: %.1f",
        m_target.getReference().getRotation().minus(m_drivetrain.getState().Pose.getRotation()).getDegrees(),
        m_target.getReference().getRotation().getDegrees(), m_drivetrain.getState().Pose.getRotation().getDegrees()));
    System.out.println(String.format("Location Difference: %.1f", m_target.getReference().getTranslation().getDistance(m_drivetrain.getState().Pose.getTranslation())));
    return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(m_request
        .withVelocityX(0)
        .withVelocityY(0));
  }

  public double getDistanceToTarget(){
    Pose2d currentPose = m_drivetrain.getState().Pose;
    double distance = currentPose.getTranslation().getDistance(m_target.getReference().getTranslation());
    return Math.abs(distance);
  }

  public double getPercentageOfDistanceToTarget(){
    return Math.abs(startingDistanceFromTarget - getDistanceToTarget()) / startingDistanceFromTarget;
  }
}