// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.AllianceSymmetry;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.RobotContainer.drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutopilotCommand extends Command {
  public final SwerveRequest.ApplyFieldSpeeds pidToPose_FieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);

      private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(4, 0, 0); /* change these values for your robot */

    private static final APConstraints kConstraints = new APConstraints()
    .withAcceleration(7.0) // example was 5
    .withJerk(3); // example was 2

    private final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Inches.of(1.5))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    public final Autopilot kAutopilot = new Autopilot(kProfile);

    private static final TrapezoidProfile.Constraints DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 5);
    // private ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, DEFAULT_CONSTRAINTS);


  double startingDistanceFromTarget; // how far from the target did we start, used to calculate when we are a percentage from the target
  Supplier<Pose2d> target;
  boolean flipPoseForAlliance;
  Optional<Rotation2d> entryAngle;

  /** Creates a new Autopilot object. responsible for instantiation. */
  public AutopilotCommand(Supplier<Pose2d> target, boolean flipPoseForAlliance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(target, flipPoseForAlliance, Optional.empty());
  }

  public AutopilotCommand(Supplier<Pose2d> target, boolean flipPoseForAlliance, Optional<Rotation2d> entryAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.flipPoseForAlliance = flipPoseForAlliance;
    this.entryAngle = entryAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDistanceFromTarget = getDistanceToTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    Pose2d correctedTargetPose = getTargetPose();
    Optional<Rotation2d> correctedEntryAngle = entryAngle;

    if (correctedEntryAngle.isPresent()) {
        correctedEntryAngle = Optional.of( correctedEntryAngle.get().plus(correctedTargetPose.getRotation()) ); // convert from input of robot relative entry angle to field relative entry angle
    }

    ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;
  

    Pose2d currentPose = drivetrain.getState().Pose;
    APResult output;
    // if (entryAngle.isPresent()){
    //     output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, new APTarget(correctedTargetPose).withEntryAngle(correctedEntryAngle.get()));
    // } else {
    //     output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, new APTarget(correctedTargetPose).withoutEntryAngle());
    // }
    output = kAutopilot.calculate(currentPose, robotRelativeSpeeds, new APTarget(correctedTargetPose).withoutEntryAngle());
    
  
    /* these speeds are field relative */
    LinearVelocity veloX = output.vx();
    LinearVelocity veloY = output.vy();
    Rotation2d headingReference = output.targetAngle();

    // double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(),
    //     correctedTargetPose.getRotation().getRadians());

    // System.out.println("thetaOutput, " + thetaOutput + ", currentRotation: " + currentPose.getRotation().getDegrees() + ", targetRotation: " + correctedTargetPose.getRotation().getDegrees());
    
    /* This is where you should apply these speeds to the drivetrain */
    drivetrain.setControl(m_request
        .withVelocityX(veloX)
        .withVelocityY(veloY)
        .withTargetDirection(headingReference));


    // if(0.2 < getPercentageOfDistanceToTarget()){
    //   // drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(veloX.magnitude(), veloY.magnitude(), thetaOutput)));
    //   drivetrain.setControl(m_request
    //     .withVelocityX(veloX)
    //     .withVelocityY(veloY)
    //     .withTargetDirection(headingReference));
    // } else {
    //   // drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(veloX.magnitude(), veloY.magnitude(), 0)));
    //   drivetrain.setControl(m_request
    //     .withVelocityX(veloX)
    //     .withVelocityY(veloY));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // thetaController.reset(0);
    drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    // drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      Pose2d currentPose = drivetrain.getState().Pose;

      Pose2d targetPose = getTargetPose();

      System.out.println(String.format("Angle Difference: %.1f, Target angle: %.1f, Current Angle: %.1f", targetPose.getRotation().minus(currentPose.getRotation()).getDegrees(), targetPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees()));

      return kAutopilot.atTarget(drivetrain.getState().Pose, new APTarget(targetPose));
  }

  public double getDistanceToTarget(){
    Pose2d currentPose = drivetrain.getState().Pose;
    double distance = currentPose.getTranslation().getDistance(getTargetPose().getTranslation());
    return Math.abs(distance);
  }

  public Pose2d getTargetPose(){
    Pose2d targetPose = target.get();
      if(flipPoseForAlliance && DriverStation.getAlliance().get().equals(Alliance.Red)) {
          targetPose = AllianceSymmetry.flip(targetPose);
      }
      return targetPose;
  }

  /**
   * 0.0 to 1.0
   * @return
   */
  public double getPercentageOfDistanceToTarget(){
    return Math.abs(startingDistanceFromTarget - getDistanceToTarget()) / startingDistanceFromTarget;
  }
}
