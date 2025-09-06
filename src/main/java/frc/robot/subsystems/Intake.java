// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CanBus;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX motor = new TalonFX(CanBus.Intake);
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final PositionVoltage holdStill = new PositionVoltage(0.0);
  private final TimeOfFlight timeOfFlight = new TimeOfFlight(CanBus.IntakeSensor);
  private final LinearFilter ampFilter = LinearFilter.movingAverage(5);
  private double cachedSensorValue = 0;
  private final BaseStatusSignal amps, voltage, velocity, position;

  private final int TOF_CORAL_THRESHOLD = 125;

  private boolean isSimulatingFeeder = false;
  private long simulatedFeederStartTime;

  // Mechanism2d visualization
  private final Mechanism2d mech2d = new Mechanism2d(3, 3); // 3x3 canvas
  private final MechanismRoot2d root = mech2d.getRoot("intakeBase", 1.5, 1.5);
  private final MechanismLigament2d coralIndicator = root.append(
    new MechanismLigament2d("coral", 0.5, 90, 5, new Color8Bit(Color.kPink))
  );
  private final MechanismLigament2d[] spokes = new MechanismLigament2d[6];
  private double flywheelRotation = 0.0;


  public Intake() {
    timeOfFlight.setRangingMode(RangingMode.Short, 24.0);

    amps = motor.getStatorCurrent(false);
    voltage = motor.getMotorVoltage(false);
    velocity = motor.getVelocity(false);
    position = motor.getPosition(false);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0,
      amps,
      voltage,
      velocity,
      position
    );
    motor.optimizeBusUtilization(4.0, 1.0);

    final TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.StatorCurrentLimit = 70.0;
    cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.kP = 20.0;

    motor.getConfigurator().apply(cfg);

    initMechanism();
    SmartDashboard.putData("IntakeMechanism", mech2d);
  }

  public void controlVolts(double volts){
    motor.setControl(voltageOut.withOutput(volts));
  }

  public double getVoltage() {
    return motor.get();
  }

  public void stop() {
    motor.setControl(holdStill.withPosition(position.getValueAsDouble()));
  }

  public boolean hasCoral(){
    return (cachedSensorValue <= TOF_CORAL_THRESHOLD);
  }

  public boolean hasAlgae() {
    return Math.abs(ampFilter.lastValue()) > 25.0;
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
      cachedSensorValue = timeOfFlight.getRange();
    }
    
    BaseStatusSignal.refreshAll(
      amps,
      voltage,
      velocity,
      position
    );
    ampFilter.calculate(amps.getValueAsDouble());

    if (Utils.isSimulation()){
      if (isSimulatingFeeder){
        if (TimeUnit.SECONDS.convert(System.nanoTime() - simulatedFeederStartTime, TimeUnit.NANOSECONDS) > 2){
          // simulated coral has entered intake
          cachedSensorValue = TOF_CORAL_THRESHOLD;
          isSimulatingFeeder = false;
        } else {
          // simulated coral has not entered intake yet
          cachedSensorValue = TOF_CORAL_THRESHOLD + 1;
        }
      } else {
        // we are in sim but not simulating a feeder scenario
        // if intake motor is running we are trying to expel a coral
        if (hasCoral() && motor.get() > 0) {
          // simulated coral has left intake
          cachedSensorValue = TOF_CORAL_THRESHOLD + 1;
        }
      }
    }

    updateMechanism();    
  }

  /**
   * Indicate that the simulation is entering the feeder.
   * Flip the indicator that will require a timer to elapse before hasCoral will return true again.
   */
  public void simulateFeeder() {
    isSimulatingFeeder = true;
    simulatedFeederStartTime = System.nanoTime();
  }

  private void initMechanism() {
    if (RobotContainer.DEBUG_CONSOLE_LOGGING){
      // Create 6 spokes spaced 60° apart
      for (int i = 0; i < spokes.length; i++) {
        double angle = i * 60.0;
        spokes[i] = root.append(new MechanismLigament2d(
          "spoke" + i, 0.8, angle, 5, new Color8Bit(Color.kBlue)
        ));
      }
    }
  }

  private void updateMechanism() {
    if (RobotContainer.DEBUG_CONSOLE_LOGGING) {
      double voltage = motor.get(); // e.g., 4.0 volts
      double spinSpeed = voltage * 10.0;

      // Update rotation angle
      flywheelRotation += spinSpeed;
      flywheelRotation %= 360;

      // Rotate all spokes
      for (int i = 0; i < spokes.length; i++) {
        double baseAngle = i * 60.0;
        spokes[i].setAngle((baseAngle + flywheelRotation) % 360);
      }

      // Show coral if detected
      coralIndicator.setLength(hasCoral() ? 0.5 : 0.0);
    }
  }
}
