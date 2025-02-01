package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class TurnToAngle extends Command{

    private static double z_P = 10.0 ;
    private static double z_I = 0.0 ;
    private static double z_D = 0.0 ;
    static{
        loadConfigs();
    }
    private ProfiledPIDController zController;
    private SwerveDrive swerve;
    private DoubleSupplier getXSpeed;
    private DoubleSupplier getYSpeed;
    private DoubleSupplier getAngle;
    

    public TurnToAngle(DoubleSupplier getXSpeed, DoubleSupplier getYSpeed, DoubleSupplier getAngle, SwerveDrive swerve) {
        addRequirements(swerve);
        
        configPIDs(swerve);
        zController.setGoal(Units.degreesToRadians(getAngle.getAsDouble()));
        this.swerve = swerve;
        this.getXSpeed = getXSpeed;
        this.getYSpeed = getYSpeed;
        this.getAngle = getAngle;
    }

    
    
    public void initialize() {
        zController.reset(swerve.getPose().getRotation().getRadians());
    }

    public void execute() {
        // update the target
        zController.setGoal(getAngle.getAsDouble());

        double angle = swerve.getPose().getRotation().getRadians();
        double rotationSpeed = zController.calculate( angle );

        swerve.drive(getXSpeed.getAsDouble(), getYSpeed.getAsDouble(), rotationSpeed, true);
    }
    

    public boolean isFinished() {
        return zController.atGoal();
      }    

    public void end(boolean interrupted) {
    }

    private void configPIDs(SwerveDrive swerve){
        zController =  new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(swerve.getMaxAngularSpeed(),swerve.getMaxAngularAcceleration()));
        zController.enableContinuousInput(-Math.PI, Math.PI); 
        zController.setTolerance((Math.PI*2)/100.0);
    } 

    private static void loadConfigs(){
        Preferences.initDouble("TurnToAngle/Z/P", z_P);
        Preferences.initDouble("TurnToAngle/Z/I", z_I);
        Preferences.initDouble("TurnToAngle/Z/D", z_D);
        z_P = Preferences.getDouble("TurnToAngle/Z/P", z_P);
        z_I = Preferences.getDouble("TurnToAngle/Z/I", z_I);
        z_D = Preferences.getDouble("TurnToAngle/Z/D", z_D);
    }

}