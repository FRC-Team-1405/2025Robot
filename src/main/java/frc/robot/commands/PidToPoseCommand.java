package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PidToPoseCommand extends Command {
    private static final TrapezoidProfile.Constraints DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 5);
    private static final TrapezoidProfile.Constraints REEF_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 6);

    private final boolean DEBUG_LOGGING_ENABLED = false;
    DataLog log = DataLogManager.getLog();
    StringLogEntry commandLog = new StringLogEntry(log, "/Commands/P2P");


    private final CommandSwerveDrivetrain drive;
    private final Supplier<Pose2d> targetPose;
    private final double toleranceInches;
    private final boolean applyFieldSymmetryToPose;
    private final double initialStateVelocity;
    private final double endStateVelocity;
    private final String commandName; // used to improve logging, if not provided targetPose is used
    private final TrapezoidProfile.Constraints drivingContraints;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private Pose2d poseToMoveTo;
    private StructPublisher<Pose2d> pidToPosePublisher = NetworkTableInstance.getDefault().getStructTopic("PID_TO_POSE/Pose", Pose2d.struct).publish();

    public static final SwerveRequest.ApplyFieldSpeeds pidToPose_FieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);

    public PidToPoseCommand(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPose, double toleranceInches, boolean applyFieldSymmetryToPose, String commandName) {
        this(drive, targetPose, toleranceInches,
                applyFieldSymmetryToPose, 0, 0, commandName, DEFAULT_CONSTRAINTS);
    }

    public PidToPoseCommand(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPose, double toleranceInches, String commandName) {
        // used in PidToPoseCommands for Reef Positions
        this(drive, targetPose, toleranceInches,
                false, 0, 0, commandName, REEF_DEFAULT_CONSTRAINTS);
    }

    public PidToPoseCommand(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPose, double toleranceInches,
            boolean applyFieldSymmetryToPose, double initialStateVelocity, double endStateVelocity) {
        this(drive, targetPose, toleranceInches,
                applyFieldSymmetryToPose, 0, 0, null, DEFAULT_CONSTRAINTS);
    }

    public PidToPoseCommand(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPose, double toleranceInches,
            boolean applyFieldSymmetryToPose, double initialStateVelocity, double endStateVelocity, String commandName, TrapezoidProfile.Constraints drivingContraints) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.toleranceInches = toleranceInches;
        this.applyFieldSymmetryToPose = applyFieldSymmetryToPose;
        this.initialStateVelocity = initialStateVelocity;
        this.endStateVelocity = endStateVelocity;
        this.commandName = commandName;
        this.drivingContraints = drivingContraints;

        xController = new ProfiledPIDController(2.2, 0, 0, drivingContraints);
        yController = new ProfiledPIDController(2.2, 0, 0, drivingContraints);
        thetaController = new ProfiledPIDController(2, 0, 0, DEFAULT_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d symmetricPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? targetPose.get()
                : AllianceSymmetry.flip(targetPose.get());
        poseToMoveTo = applyFieldSymmetryToPose ? symmetricPose : targetPose.get();
        pidToPosePublisher.set(poseToMoveTo);

        log("Target pose: " + targetPose);
        log("Pose to move to (after symmetry): " + poseToMoveTo);
        log("Current pose: " + drive.getState().Pose);

        // Without resetting when we chain commands together the robot will drive full speed in a weird direction. unclear why but this fixes it.
        Pose2d currentPose = drive.getState().Pose;
        xController.reset(new TrapezoidProfile.State(currentPose.getX(), initialStateVelocity));
        yController.reset(new TrapezoidProfile.State(currentPose.getY(), initialStateVelocity));
        thetaController.reset(new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0)); // TODO: provide a value?

        commandLog.append("Initialized: " + getName());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;

        double xOutput = xController.calculate(currentPose.getX(),
                new TrapezoidProfile.State(poseToMoveTo.getX(), endStateVelocity));
        double yOutput = yController.calculate(currentPose.getY(),
                new TrapezoidProfile.State(poseToMoveTo.getY(), endStateVelocity));
        double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(),
                poseToMoveTo.getRotation().getRadians());

        SmartDashboard.putNumber("PID_TO_POSE/xError", xController.getPositionError());
        SmartDashboard.putNumber("PID_TO_POSE/yError", yController.getPositionError());
        SmartDashboard.putNumber("PID_TO_POSE/xOutput", xOutput);
        SmartDashboard.putNumber("PID_TO_POSE/yOutput", yOutput);
        SmartDashboard.putNumber("PID_TO_POSE/thetaOutput", thetaOutput);

        drive.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(xOutput, yOutput, thetaOutput)));
    }

    @Override
    public boolean isFinished() {
        double distance = drive.getState().Pose.getTranslation().getDistance(poseToMoveTo.getTranslation());
        return Units.metersToInches(distance) < toleranceInches;
    }

    @Override
    public void end(boolean interrupted) {
        //TODO modify command to optionally apply "brakes" where it not only cuts velocity to wheels but moves the wheels into brake formation making the robot harder to move
        if (endStateVelocity == 0) {
            log("KILLING DRIVE VELOCITY");
            drive.setControl(pidToPose_FieldSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));

            SmartDashboard.putNumber("PID_TO_POSE/xError", xController.getPositionError());
            SmartDashboard.putNumber("PID_TO_POSE/yError", yController.getPositionError());
        }
        
        double finalDistance = Units
                .metersToInches(drive.getState().Pose.getTranslation().getDistance(poseToMoveTo.getTranslation()));
        log("PidToPose " + (interrupted ? "interrupted" : "completed") +
                ". Final pose: " + drive.getState().Pose +
                ", Target: " + poseToMoveTo +
                ", Distance to target (in): " + finalDistance +
                ", End state velocity: ( x: " + drive.getState().Speeds.vxMetersPerSecond + ", y: " + drive.getState().Speeds.vyMetersPerSecond + " )");

        commandLog.append("Finished (interrupt: " + (interrupted ? "Y" : "N") + "): " + getName());
    }

    private void log(String logMessage) {
        if (DEBUG_LOGGING_ENABLED) {
            DataLogManager.log("[PidToPoseCommand] " + logMessage);
        }
    }

    @Override
    public String getName() {
        String instanceSpecificValue = commandName == null ? formatPose(targetPose.get()) : commandName;
        return "PidToPoseCommand(" + instanceSpecificValue + ")";
    }

    private String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)", 
            pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

}