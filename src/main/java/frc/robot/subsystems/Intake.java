// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBus;
import frc.robot.lib.FusionTimeofFlight;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX primary = new TalonFX(CanBus.Intake);
  private final double CoralSpeed;
  private final double AlgeaSpeed;
  private final FusionTimeofFlight timeofFlight = new FusionTimeofFlight(CanBus.IntakeSensor);
  private final Supplier<ForwardLimitValue> reefDetector = primary.getForwardLimit().asSupplier();
  private double sensorValue = 0;

  public Intake() {
      Preferences.initDouble("Intake/CoralSpeed", 0.25);
      CoralSpeed = Preferences.getDouble("Elavator/Position/", 0.25);
      Preferences.initDouble("Intake/AlgeaSpeed", 0.25);
      AlgeaSpeed = Preferences.getDouble("Elavator/Position/", -0.25);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sensorValue = timeofFlight.Measure();
  }

  public void stop(){
    primary.set(0.0);
  }

  public void intakeCoral(){
    primary.set(CoralSpeed);
  }

  public void outtakeCoral(){
    primary.set(-CoralSpeed);
  }

  public boolean hasCoral(){
    return (sensorValue <= 100);
  }

  public void intakeAlgae(){
    primary.set(AlgeaSpeed);
  }

  public void outtakeAlgae(){
    primary.set(-AlgeaSpeed);
  }

  public boolean reefDetected() {
    return reefDetector.get() == ForwardLimitValue.ClosedToGround;
  }
}
