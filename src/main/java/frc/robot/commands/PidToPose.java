// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class PidToPose {

//     public static Command PidToPose(CommandSwerveDrivetrain swerveDrivetrain, Pose2d targetpose) {
//         PIDController xcontroller = new PIDController(2, 0, 0);  //TODO use profilePID
//         PIDController ycontroller = new PIDController(2, 0, 0);  //TODO use profilePID

//     return swerveDrivetrain.run( () ->
//         var speeds = fieldspeeds(
//             xcontroller.calculate(currentpose.getx(), targetpose.getx()),
//             ycontroller.calculate(currentpose.gety(), targetpose.gety()),
//             thetacontroller.calculate()
//         );

//         swerve.drive(speeds);
//     ).until(
//         //until you are close enough 0.04meter tolerance?
//     );
//     }
        
// }
