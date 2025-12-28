// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PidToPose;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MoveCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevationLevel;
import frc.robot.subsystems.Intake;

public class PidToPoseAutoScore {
  public static Command runAutoScore(CommandSwerveDrivetrain drivetrain, Supplier<Optional<Pose2d>> targetPoseSupplier, Supplier<ElevationLevel> elevationLevelSupplier, Intake intake, Elevator elevator, Arm arm) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    if (RobotContainer.DEBUG_CONSOLE_LOGGING) {
                        System.out.println("runAutoAlign Called with targetPose: " + targetPoseSupplier.get());
                    }
                }),
                Commands.defer(() -> {
                    Optional<Pose2d> targetPose = targetPoseSupplier.get();

                    if (targetPose.isEmpty()) {
                        return Commands.none();
                    }

                    return new PidToPoseCommand(drivetrain, () -> targetPose.get(), 1.2, false, null)
                    .alongWith(
                                        Commands.sequence(
                                                Commands.waitUntil(() -> drivetrain.getState().Pose.getTranslation()
                                                        .getDistance(targetPose.get().getTranslation()) < 1)).andThen(
                                                            () -> {
                                                                System.out.println("PidToPose is within a meter of target, start moving elevator to level: " + elevationLevelSupplier.get());
                                                            }
                                                        ),
                                        new MoveCoral(elevator, arm, elevationLevelSupplier, intake))
                                        .andThen(() -> {System.out.println("runAutoAlign is in position to score, deploying coral now.");})
                                .andThen(
                                        new ParallelRaceGroup(
                                                IntakeCommands.expelCoral(intake),
                                                new ArmPosition(arm, () -> ArmLevel.Travel)
                                                        .beforeStarting(Commands.waitSeconds(0.25))));
                }, Set.of(drivetrain)));
    }
}
