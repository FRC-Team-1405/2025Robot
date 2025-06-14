// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBus;
import com.playingwithfusion.TimeOfFlight;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX primary = new TalonFX(CanBus.Intake);
  private final VelocityDutyCycle velocityVoltage = new VelocityDutyCycle(0).withSlot(1);
  private final double CoralImputSpeed;
  private final double CoralOutputSpeed;
  private final double AlgeaSpeed;
  private final TimeOfFlight timeofFlight = new TimeOfFlight(CanBus.IntakeSensor);
  private final Supplier<ForwardLimitValue> reefDetector = primary.getForwardLimit().asSupplier();
  private double sensorValue = 0;

  public Intake() {
      Preferences.initDouble("Intake/CoralImputSpeed", -15.0);
      CoralImputSpeed = Preferences.getDouble("Intake/CoralImputSpeed/", -15.0);

      Preferences.initDouble("Output/CoralOutputSpeed", -30.0);
      CoralOutputSpeed = Preferences.getDouble("Output/CoralOutputSpeed/", -30.0);
      
      Preferences.initDouble("Intake/AlgeaSpeed", 0.25);
      AlgeaSpeed = Preferences.getDouble("Elavator/Position/", -0.25);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sensorValue = timeofFlight.getRange();
    SmartDashboard.putBoolean("Intake/HaveCoral", hasCoral());
    SmartDashboard.putBoolean("Intake/HitReef", reefDetected());
    if (DriverStation.isTeleopEnabled()){
      System.out.println("intake current: " + primary.getStatorCurrent().getValueAsDouble());
    }
  }

  public void stop(){
    primary.set(0.0);
  }
  
  public void pushCoral(){
    primary.setPosition(primary.getPosition().getValue().in(Rotations)+1);
  }
  public void pullCoral(){
    primary.setPosition(primary.getPosition().getValue().in(Rotations)-1);

  }
  public void intakeCoral(){
    primary.setControl(velocityVoltage.withVelocity(CoralImputSpeed));
  }

  public void slowIntakeCoral() {
    primary.setControl(velocityVoltage.withVelocity(CoralImputSpeed/2));
  }

  public void outtakeCoral(){
    primary.setControl(velocityVoltage.withVelocity(CoralOutputSpeed));
  }
  public void slowScore(){
    primary.setControl(velocityVoltage.withVelocity(-CoralImputSpeed));
  }

  public double getCurrent() {
    return primary.getStatorCurrent().getValueAsDouble();
  }

 

  public boolean hasCoral(){
    return (sensorValue <= 100);
  }

  public void intakeAlgae(){
    primary.set(-AlgeaSpeed);
  }

  public void outtakeAlgae(){
    primary.set(AlgeaSpeed);
  }

  public boolean reefDetected() {
    return reefDetector.get() == ForwardLimitValue.ClosedToGround;
  }
}
