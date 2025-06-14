// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NewCoralInput extends Command {
  /** Creates a new CoralOutput. */
  private Intake intake;
  private int cycles = 0;
  private boolean coralIntakeStarted = false;
  private long startTime;
  private long endTime;
  public NewCoralInput(Intake intake) {
    this.intake = intake;
    cycles = 0;
    coralIntakeStarted = false;
//    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cycles = 0;
    coralIntakeStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (intake.getCurrent() > 10){
        System.out.println("CoralIsBeingIntaked");
        if (!coralIntakeStarted) {
            coralIntakeStarted = true;
            startTime = System.nanoTime();
        }
      }

      if (coralIntakeStarted) {
        cycles++;

        if (cycles > 5) {
            intake.slowIntakeCoral();
            System.out.println("Slowing the coral intake down.");
        }
      }

      if (!intake.hasCoral()){
        intake.intakeCoral();
      } else {
        endTime = System.nanoTime();
        System.out.println("number of cycles before sensor triggered = " + cycles);
        System.out.println("number of ms before sensor triggered = " + (endTime - startTime) / 1_000_000);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasCoral();
  }
}
