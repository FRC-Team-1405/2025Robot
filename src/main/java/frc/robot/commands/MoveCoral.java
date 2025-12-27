// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ArmLevel;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveCoral extends SequentialCommandGroup {

  private Supplier<Elevator.ElevationLevel> level;

  public MoveCoral(Elevator elevator, Supplier<Elevator.ElevationLevel> level, Intake intake) {
    this.level = level;

    addRequirements(elevator);
    // addCommands(IntakeCommands.stopIntake(intake)); TODO determine how to add this back in without error: IllegalArgumentException: Multiple commands in a parallel composition cannot require the same subsystems
    
    BooleanSupplier alreadyAtLevel = () -> elevator.isAtLevel(level.get());

    // TODO separate the arm from the elevator so you can move the arm at the same time as the elevator
    // Fast Move is moving the elevator and the arm at the same time
    // BooleanSupplier allowFastMove = () -> elevator.isArmSafeToTravel();

    Command moveToLevelSequence =
        new SequentialCommandGroup(
            new ArmPosition(elevator, () -> ArmLevel.Travel),
            new MoveElevator(elevator, level)
        );

    // Command moveToLevelSequence_fastMove =
    // new ParallelCommandGroup(
    //     new ArmPosition(elevator, () -> ArmLevel.Travel),
    //     new MoveElevator(elevator, level)
    // );

    // if elevator needs to move do a normal move
    Command conditionalMove =
        new ConditionalCommand(
            new InstantCommand(),   // If true: do nothing
            moveToLevelSequence,    // If false: run the sequence
            alreadyAtLevel
        );

    // if elevator needs to move do a fast move
    // Command conditionalMove_fastMove =
    // new ConditionalCommand(
    //     new InstantCommand(),
    //     moveToLevelSequence_fastMove,
    //     alreadyAtLevel
    // );

    // Command conditionalElevatorMove_possibleFastMovement = new ConditionalCommand(conditionalMove_fastMove, conditionalMove, allowFastMove);

    Command finalArmMove = new ArmPosition(elevator, this::armLevel);

    // Build the full command sequence
    addCommands(
      conditionalMove,
        finalArmMove
    );

    this.setName("MoveCoral");
  }

  private ArmLevel armLevel() {
    ArmLevel value = switch ((level.get())) {
      case Home -> ArmLevel.Home;
      case Level_1 -> ArmLevel.Low_Score;
      case Level_2 -> ArmLevel.Middle_Score;
      case Level_3 -> ArmLevel.Middle_Score;
      case Level_4 -> ArmLevel.High_Score;
      case Level_4_Auto -> ArmLevel.High_Score_Auto;
      default -> ArmLevel.Home;
    };
    return value;
  }
}
