// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elavator;
import frc.robot.subsystems.Elavator.ArmLevel;
import frc.robot.subsystems.Elavator.ElevationLevel;
import frc.robot.subsystems.Intake;

public class GrabAlgae extends SequentialCommandGroup {
  /** Creates a new GrabAlgae. */
  public GrabAlgae(Elavator elevator, Intake intake, BooleanSupplier highAlgae) {
    addRequirements(elevator);
    addRequirements(intake);
    addCommands( new ArmPosition(elevator, () -> ArmLevel.Algae));
    addCommands( new MoveElevator(elevator, () -> { return highAlgae.getAsBoolean() ? ElevationLevel.Level_3 :ElevationLevel.Level_2; }));
    addCommands( intake.runOnce( intake::intakeAlgae ));
  }
}
