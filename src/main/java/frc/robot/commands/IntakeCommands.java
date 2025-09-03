package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

@Logged
public class IntakeCommands {
    public static String INTAKE_CORAL = "Intake Coral";
    
    /**
     * Creates a trigger following the hasCoral method of the Intake subsystem.
     *
     * @param intake The intake subsystem.
     * @return A trigger that is true when the intake has coral.
     */
    public static Trigger hasCoral(Intake intake) {
        return new Trigger(intake::hasCoral);
    }

    /**
     * Creates a trigger following the hasAlgae method of the Intake subsystem.
     *
     * @param intake The intake subsystem.
     * @return A trigger that is true when the intake has algae.
     */
    public static Trigger hasAlgae(Intake intake) {
        return new Trigger(intake::hasAlgae);
    }

    /**
     * Runs the intake at a specified voltage.
     *
     * @param intake The intake subsystem.
     * @param volts The voltage to run the intake at.
     * @return A command that runs the intake at the specified voltage.
     */
    public static Command runVoltage(Intake intake, double volts) {
        return intake.run(() -> intake.controlVolts(volts))
            .withName("IntakeVolts(" + volts + ")");
    }

    public static Command stopIntake(Intake intake) {
        return intake.runOnce(() -> intake.controlVolts(0.0))
            .withName("Stop Intake");
    }

    /**
     * Runs the intake and stops it when a coral is detected
     *
     * @param intake The intake subsystem.
     * @return A command that runs the intake until a coral is detected.
     */
    public static Command intakeCoral(Intake intake) {
    Command indicateFeederIfSim = new InstantCommand(() -> intake.simulateFeeder())
        .unless(() -> !Utils.isSimulation());

    Command coralIntake = intake.run(() -> intake.controlVolts(3.0))
        .finallyDo(() -> intake.stop())
        .until(hasCoral(intake))
        .unless(hasAlgae(intake).or(hasCoral(intake)))
        .withName(INTAKE_CORAL);

    return new SequentialCommandGroup(indicateFeederIfSim, coralIntake);
}

    /**
     * Runs the intake until no coral is detected and a period of time has passed.
     *
     * @param intake The intake subsystem.
     * @return A command that runs the intake until no coral is detected and a period of time has passed.
     */
    public static Command expelCoral(Intake intake) {
        return Commands.deadline(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.waitUntil(hasCoral(intake).negate())
            ),
            runVoltage(intake, 4.0)
        ).finallyDo(() -> intake.controlVolts(0.0))
        .unless(hasAlgae(intake))
        .withName("Expel Coral");
    }
}
