package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

    // initial velocity mechanism
    Mechanism2d initialMechanism;
    MechanismRoot2d initialOrigin;

    MechanismLigament2d initialDeltaLigament;
    MechanismLigament2d initialDirectionLigament;
    MechanismLigament2d initialVelocityLigament;

    // execution velocity mechanism
    Mechanism2d loopMechanism;
    MechanismRoot2d loopOrigin;

    MechanismLigament2d loopDeltaLigament;
    MechanismLigament2d loopDirectionLigament;
    MechanismLigament2d loopVelocityLigament;


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

        initialMechanism = new Mechanism2d(2, 2); // 2x2 unit canvas
        initialOrigin = initialMechanism.getRoot("initialOrigin", 1, 1); // center of canvas

        initialDeltaLigament = initialOrigin.append(new MechanismLigament2d("initialdelta", 0, 0, 1, new Color8Bit(Color.kBlue)));
        initialDirectionLigament = initialOrigin.append(new MechanismLigament2d("initialdirection", 0, 0, 1, new Color8Bit(Color.kGreen)));
        initialVelocityLigament = initialOrigin.append(new MechanismLigament2d("initialvelocity", 0, 0, 1, new Color8Bit(Color.kRed)));


        loopMechanism = new Mechanism2d(2, 2); // 2x2 unit canvas
        loopOrigin = loopMechanism.getRoot("loopOrigin", 1, 1); // center of canvas

        loopDeltaLigament = loopOrigin.append(new MechanismLigament2d("loopdelta", 0, 0, 1, new Color8Bit(Color.kBlue)));
        loopDirectionLigament = loopOrigin.append(new MechanismLigament2d("loopdirection", 0, 0, 1, new Color8Bit(Color.kGreen)));
        loopVelocityLigament = loopOrigin.append(new MechanismLigament2d("loopvelocity", 0, 0, 1, new Color8Bit(Color.kRed)));

        SmartDashboard.putData("InitialVectorMechanism", initialMechanism);
        SmartDashboard.putData("LoopVectorMechanism", loopMechanism);

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

        Translation2d delta = poseToMoveTo.getTranslation().minus(currentPose.getTranslation());
        Translation2d direction = delta.div(delta.getNorm()); // get a unit vector
        Translation2d initialVelocityVector = direction.times(initialStateVelocity);

        // MECHANISM

        double deltaLength = delta.getNorm();
        double deltaAngle = delta.getAngle().getDegrees(); // blue

        double directionLength = 1.0; // unit vector
        double directionAngle = direction.getAngle().getDegrees(); // green

        double velocityLength = initialVelocityVector.getNorm();
        double velocityAngle = initialVelocityVector.getAngle().getDegrees(); // red

        initialDeltaLigament.setLength(deltaLength);
        initialDeltaLigament.setAngle(deltaAngle);

        initialDirectionLigament.setLength(directionLength);
        initialDirectionLigament.setAngle(directionAngle);

        initialVelocityLigament.setLength(velocityLength);
        initialVelocityLigament.setAngle(velocityAngle);

        // MECHANISM

        // if (applyFieldSymmetryToPose && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        //     initialVelocityVector = AllianceSymmetry.flip(initialVelocityVector);
        // }

        xController.reset(new TrapezoidProfile.State(currentPose.getX(), initialVelocityVector.getX()));
        yController.reset(new TrapezoidProfile.State(currentPose.getY(), initialVelocityVector.getY()));
        thetaController.reset(new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0)); // TODO: provide a value?

        commandLog.append("Initialized: " + getName());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;

        Translation2d delta = poseToMoveTo.getTranslation().minus(currentPose.getTranslation());
        Translation2d direction = delta.div(delta.getNorm()); // get a unit vector
        Translation2d endVelocityVector = direction.times(endStateVelocity);

        // if (applyFieldSymmetryToPose && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        //     endVelocityVector = AllianceSymmetry.flip(endVelocityVector);
        // }

        // MECHANISM

        double deltaLength = delta.getNorm();
        double deltaAngle = delta.getAngle().getDegrees(); // blue

        double directionLength = 1.0; // unit vector
        double directionAngle = direction.getAngle().getDegrees(); // green

        double velocityLength = endVelocityVector.getNorm();
        double velocityAngle = endVelocityVector.getAngle().getDegrees(); // red

        loopDeltaLigament.setLength(deltaLength);
        loopDeltaLigament.setAngle(deltaAngle);

        loopDirectionLigament.setLength(directionLength);
        loopDirectionLigament.setAngle(directionAngle);

        loopVelocityLigament.setLength(velocityLength);
        loopVelocityLigament.setAngle(velocityAngle);

        // MECHANISM

        double xOutput = xController.calculate(currentPose.getX(),
                new TrapezoidProfile.State(poseToMoveTo.getX(), endVelocityVector.getX()));
        double yOutput = yController.calculate(currentPose.getY(),
                new TrapezoidProfile.State(poseToMoveTo.getY(), endVelocityVector.getY()));
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