package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class PidToPoseCommand extends Command {
    private final boolean DEBUG_LOGGING_ENABLED = false;

    private final CommandSwerveDrivetrain drive;
    private final Pose2d targetPose;
    private final double toleranceInches;
    private final boolean applyFieldSymmetryToPose;
    private final double endStateVelocity;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final PIDController thetaController;

    private Pose2d poseToMoveTo;

    public PidToPoseCommand(CommandSwerveDrivetrain drive, Pose2d targetPose, double toleranceInches,
            boolean applyFieldSymmetryToPose, double endStateVelocity) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.toleranceInches = toleranceInches;
        this.applyFieldSymmetryToPose = applyFieldSymmetryToPose;
        this.endStateVelocity = endStateVelocity;

        xController = new ProfiledPIDController(2.2, 0, 0, new TrapezoidProfile.Constraints(4, 5));
        yController = new ProfiledPIDController(2.2, 0, 0, new TrapezoidProfile.Constraints(4, 5));
        thetaController = new PIDController(2, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d symmetricPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? targetPose
                : AllianceSymmetry.flip(targetPose);
        poseToMoveTo = applyFieldSymmetryToPose ? symmetricPose : targetPose;

        log("Target pose: " + targetPose);
        log("Pose to move to (after symmetry): " + poseToMoveTo);
        log("Current pose: " + drive.getState().Pose);

        // Without resetting when we chain commands together the robot will drive full speed in a weird direction. unclear why but this fixes it.
        xController.reset(drive.getState().Pose.getX());
        yController.reset(drive.getState().Pose.getY());
        thetaController.reset(); // TODO: provide a value?
    }

    @Override
    public void execute() {
        drive.applyRequest(() -> {
            Pose2d currentPose = drive.getState().Pose;

            log("Current pose: " + currentPose);

            double xOutput = xController.calculate(currentPose.getX(),
                    new TrapezoidProfile.State(poseToMoveTo.getX(), endStateVelocity));
            double yOutput = yController.calculate(currentPose.getY(),
                    new TrapezoidProfile.State(poseToMoveTo.getY(), endStateVelocity));
            double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(),
                    poseToMoveTo.getRotation().getRadians());

            log("Raw outputs — X: " + xOutput + ", Y: " + yOutput + ", Theta: " + thetaOutput);

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                xOutput = -xOutput;
                yOutput = -yOutput;
                log("Flipped outputs for Red alliance — X: " + xOutput + ", Y: " + yOutput);
            }

            SmartDashboard.putNumber("PID_TO_POSE/xError", xController.getPositionError());
            SmartDashboard.putNumber("PID_TO_POSE/yError", yController.getPositionError());
            SmartDashboard.putNumber("PID_TO_POSE/xOutput", xOutput);
            SmartDashboard.putNumber("PID_TO_POSE/yOutput", yOutput);
            SmartDashboard.putNumber("PID_TO_POSE/thetaOutput", thetaOutput);

            log("Applying velocities — X: " + xOutput + ", Y: " + yOutput
                    + ", Theta: " + thetaOutput);

            return RobotContainer.pidToPose_FieldCentricDrive.withVelocityX(xOutput)
                    .withVelocityY(yOutput)
                    .withRotationalRate(thetaOutput);
        }).execute();
    }

    @Override
    public boolean isFinished() {
        double distance = drive.getState().Pose.getTranslation().getDistance(poseToMoveTo.getTranslation());
        return Units.metersToInches(distance) < toleranceInches;
    }

    @Override
    public void end(boolean interrupted) {
        // drive.stop(); TODO: do i want to brake? probably not

        double finalDistance = Units
                .metersToInches(drive.getState().Pose.getTranslation().getDistance(poseToMoveTo.getTranslation()));
        log("PidToPose " + (interrupted ? "interrupted" : "completed") +
                ". Final pose: " + drive.getState().Pose +
                ", Target: " + poseToMoveTo +
                ", Distance to target (in): " + finalDistance);
    }

    private void log(String logMessage) {
        if (DEBUG_LOGGING_ENABLED) {
            System.out.println("[PidToPoseCommand] " + logMessage);
        }
    }
}