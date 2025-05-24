// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PidToPoseCommands {
  public static Pose2d Red_5B = new Pose2d(5.270, 3.000, Rotation2d.fromDegrees(-60));
  public static Pose2d Red_5A = new Pose2d(4.98, 2.84, Rotation2d.fromDegrees( -60));
  public static Pose2d Red_6B = new Pose2d(3.99, 2.84, Rotation2d.fromDegrees(-120.0));

  public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
    Command MoveTo_Red_5B = CommandSwerveDrivetrain.PidToPose(drivetrain, Red_5B, 1);
    Command MoveTo_Red_5A = CommandSwerveDrivetrain.PidToPose(drivetrain, Red_5A, 1);
    Command MoveTo_Red_6B = CommandSwerveDrivetrain.PidToPose(drivetrain, Red_6B, 1);

    NamedCommands.registerCommand("MoveTo_Red_5B", MoveTo_Red_5B);
    NamedCommands.registerCommand("MoveTo_Red_5A", MoveTo_Red_5A);
    NamedCommands.registerCommand("MoveTo_Red_6B", MoveTo_Red_6B);
  }
}
