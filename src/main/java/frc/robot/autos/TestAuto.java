package frc.robot.autos;

import java.io.IOException;
import java.util.HashMap;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestAuto {
    public static void configureAutos(SendableChooser<Command> chooser, CommandSwerveDrivetrain drivetrain) {
        HashMap<String, Command> commandsToAddToChooser = new HashMap<>();

        //region PidToPose
        commandsToAddToChooser.put("P2P_DS_Right_3Piece_WaitIntake", NamedCommands.getCommand("P2P_DS_Right_3Piece_WaitIntake"));
        commandsToAddToChooser.put("P2P_DS_Right_3Piece_ParallelIntake", NamedCommands.getCommand("P2P_DS_Right_3Piece_ParallelIntake"));

        commandsToAddToChooser.put("P2P_DS_Left_3Piece_ParallelIntake", NamedCommands.getCommand("P2P_DS_Left_3Piece_ParallelIntake"));

        commandsToAddToChooser.put("P2P_DS_Center", NamedCommands.getCommand("P2P_DS_Center"));
        //region PidToPose


        //region autopilot
        commandsToAddToChooser.put("AP_DS_Right_3Piece_WaitIntake", NamedCommands.getCommand("AP_DS_Right_3Piece_WaitIntake"));
        commandsToAddToChooser.put("AP_DS_Right_3Piece_ParallelIntake", NamedCommands.getCommand("AP_DS_Right_3Piece_ParallelIntake"));

        commandsToAddToChooser.put("AP_DS_Left_3Piece_WaitIntake", NamedCommands.getCommand("AP_DS_Left_3Piece_WaitIntake"));
        commandsToAddToChooser.put("AP_DS_Left_3Piece_ParallelIntake", NamedCommands.getCommand("AP_DS_Left_3Piece_ParallelIntake"));

        commandsToAddToChooser.put("AP_DS_Center", NamedCommands.getCommand("AP_DS_Center"));
        //region autopilot


        chooser.setDefaultOption("P2P_DS_Right_3Piece_WaitIntake", commandsToAddToChooser.get("P2P_DS_Right_3Piece_WaitIntake"));

        // Add all commands in Map to chooser
        commandsToAddToChooser.keySet().stream().forEach(name -> chooser.addOption(name, commandsToAddToChooser.get(name)));
    }

    private static Command runPathPlannerPath(String pathName, CommandSwerveDrivetrain drivetrain, boolean resetOdometry) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return new SequentialCommandGroup(
                Commands.runOnce(() -> drivetrain.resetPose(path.getStartingDifferentialPose())).onlyIf(() -> resetOdometry),
                AutoBuilder.followPath(path)
                );
        } catch (FileVersionException | IOException | ParseException e) {
            System.err.println("ERROR: PathPlannerPath not found, name: " + pathName);
            System.err.println(e.getStackTrace());
            return null;
        }
    }
}
