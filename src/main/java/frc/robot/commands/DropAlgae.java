// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Elevator.ElevationLevel;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropAlgae extends SequentialCommandGroup {
  /** Creates a new DropAlgae. */
  public DropAlgae(Elevator elevator, Intake intake) {
    addRequirements(elevator);
    addCommands( intake.runOnce( intake::outtakeAlgae ));
    addCommands( new ArmPosition(elevator, () -> ArmLevel.Algae));
    addCommands( new ArmPosition(elevator, () -> ArmLevel.Algae_Output));
    addCommands( intake.runOnce( intake::stop ));
    addCommands( new ArmPosition(elevator, () -> ArmLevel.Travel));
    addCommands( new MoveElevator(elevator, () -> ElevationLevel.Home));
    addCommands( new ArmPosition(elevator, () -> ArmLevel.Home));
  }
}
