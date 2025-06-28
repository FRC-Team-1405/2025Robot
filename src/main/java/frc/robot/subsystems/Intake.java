// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  }

  public void controlVolts(double volts){
    motor.setControl(voltageOut.withOutput(volts));
  }

  public void stop() {
    motor.setControl(holdStill.withPosition(position.getValueAsDouble()));
  }

  public boolean hasCoral(){
    return (cachedSensorValue <= 125);
  }

  public boolean hasAlgae() {
    return Math.abs(ampFilter.lastValue()) > 25.0;
  }

  @Override
  public void periodic() {
    cachedSensorValue = timeOfFlight.getRange();
    BaseStatusSignal.refreshAll(
      amps,
      voltage,
      velocity,
      position
    );
    ampFilter.calculate(amps.getValueAsDouble());
  }
}
