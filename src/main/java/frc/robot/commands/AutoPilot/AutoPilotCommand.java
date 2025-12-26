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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.lib.FinneyCommand;
import frc.robot.lib.FinneyLogger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPilotCommand extends FinneyCommand {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  // Publishes AP's target position
  private StructPublisher<Pose2d> apPublisher = NetworkTableInstance.getDefault().getStructTopic("AUTOPILOT/Pose", Pose2d.struct).publish();

  public final SwerveRequest.ApplyFieldSpeeds pidToPose_FieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);
  public final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  private static final APConstraints kConstraints = new APConstraints()
      .withAcceleration(5.0)
      .withJerk(2.0);

  // AutoPilot Thresholds
  private static final APProfile kProfile = new APProfile(kConstraints)
      .withErrorXY(Centimeters.of(2))
      .withErrorTheta(Degrees.of(0.5))
      .withBeelineRadius(Centimeters.of(16));

  public static final Autopilot kAutopilot = new Autopilot(kProfile);

  private APTarget m_target;
  private final Supplier<Pose2d> m_targetSupplier;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Optional<Rotation2d> m_entryAngle;
  private final boolean m_flipPoseForAlliance;
  private double startingDistanceFromTarget;
  private Pose2d startingPosition;
  private final String commandName;

  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(
    2.0, 0.0, 0.0, // PID gains
    new TrapezoidProfile.Constraints(Math.PI, Math.PI) // max velocity and acceleration
);

  private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withHeadingPID(2, 0, 0); /* tune this for your robot! */


  public AutoPilotCommand(Supplier<Pose2d> target, CommandSwerveDrivetrain drivetrain, String commandName) {
    this(target, drivetrain, Optional.empty(), false, commandName);
  }

  public AutoPilotCommand(Supplier<Pose2d> target, CommandSwerveDrivetrain drivetrain, Optional<Rotation2d> entryAngle, String commandName) {
    this(target, drivetrain, entryAngle, false, commandName);
  }

  public AutoPilotCommand(Supplier<Pose2d> target, CommandSwerveDrivetrain drivetrain, Optional<Rotation2d> entryAngle, boolean flipPoseForAlliance, String commandName) {
    m_targetSupplier = target;
    m_drivetrain = drivetrain;
    m_entryAngle = entryAngle;
    m_flipPoseForAlliance = flipPoseForAlliance;
    this.commandName = commandName;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    super.initialize();
    /* no-op */
    if (m_flipPoseForAlliance && DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())) {
      // flip pose for red alliance
      m_target = new APTarget(AllianceSymmetry.flip(m_targetSupplier.get()));
    } else {
      m_target = new APTarget(m_targetSupplier.get());
    }
    apPublisher.set(m_target.getReference());
    
    startingDistanceFromTarget = getDistanceToTarget();
    startingPosition = m_drivetrain.getState().Pose;

    m_thetaController.reset(startingPosition.getRotation().getRadians());

    fLogger.log("Initializing %s to target Pose (x: %.1f, y: %.1f, rot: %.1f deg)",
      getName(),
      m_target.getReference().getX(), m_target.getReference().getY(), m_target.getReference().getRotation().getDegrees());
  }

  @Override
  public void execute() {
    ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getState().Speeds;
    Pose2d pose = m_drivetrain.getState().Pose;
    // System.out.println(String.format("Robot Pose (x: %.1f, y: %.1f)", pose.getX(), pose.getY()));

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
      // Target approach section. at the end of motion towards target
      // custom theta PID controller used to ensure we reach target rotation

      // drivetrain.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(veloX.magnitude(), veloY.magnitude(), thetaOutput)));
      
      fLogger.log("vx: %.3f, vy: %.3f, thetaOutput: %.3f, rotationDifference(deg): %.2f", out.vx().baseUnitMagnitude(), out.vy().baseUnitMagnitude(), thetaOutput, (m_target.getReference().getRotation().getDegrees() - pose.getRotation().getDegrees()));
      // System.out.println("percentageToTarget: +60%");
      m_drivetrain.setControl(applyRobotSpeeds.withSpeeds(outRobotRelativeSpeeds));
    } else if(0.2 < getPercentageOfDistanceToTarget()){
      // System.out.println("percentageToTarget: +20%");
      m_drivetrain.setControl(m_request
        .withVelocityX(out.vx())
        .withVelocityY(out.vy())
        .withTargetDirection(targetRotation));
    } else {
      // Beginning of motion towards target, don't start rotation yet
      // to allow for movement away from walls before rotation begins.

      // fLogger.log("vx: %.3f, vy: %.3f, thetaOutput: %.3f, rotationDifference(deg): %.2f", out.vx().baseUnitMagnitude(), out.vy().baseUnitMagnitude(), thetaOutput, (m_target.getReference().getRotation().getDegrees() - pose.getRotation().getDegrees()));
      m_drivetrain.setControl(m_request
        .withVelocityX(out.vx())
        .withVelocityY(out.vy())
        .withTargetDirection(startingPosition.getRotation()));
    }
  }

  @Override
  public boolean isFinished() {
    // SystfLoggerem.out.println(String.format("Angle Difference: %.1f, Target angle: %.1f, Current Angle: %.1f",
    //     m_target.getReference().getRotation().minus(m_drivetrain.getState().Pose.getRotation()).getDegrees(),
    //     m_target.getReference().getRotation().getDegrees(), m_drivetrain.getState().Pose.getRotation().getDegrees()));
    // .log(String.format("Location Difference: %.1f, Angle Difference: %.1f", m_target.getReference().getTranslation().getDistance(m_drivetrain.getState().Pose.getTranslation()), m_target.getReference().getRotation().minus(m_drivetrain.getState().Pose.getRotation()).getDegrees()));
    return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drivetrain.setControl(m_request
        .withVelocityX(0)
        .withVelocityY(0)
        .withTargetDirection(m_drivetrain.getState().Pose.getRotation()));

    fLogger.log("%s ended, final Pose (x: %.1f, y: %.1f, rot: %.1f deg), target Pose (x: %.1f, y: %.1f, rot: %.1f deg), interrupted: %s",
        getName(),
        m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY(), m_drivetrain.getState().Pose.getRotation().getDegrees(),
        m_target.getReference().getX(), m_target.getReference().getY(), m_target.getReference().getRotation().getDegrees(),
        interrupted);
  }

  @Override
    public String getName() {
        String instanceSpecificValue = commandName;
        return "AutoPilot(" + instanceSpecificValue + ")";
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