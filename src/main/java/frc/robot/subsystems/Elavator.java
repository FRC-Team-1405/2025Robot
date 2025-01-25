// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.Preferences;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.StatusSignalJNI;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanID;
import frc.robot.Constants.ElavationConstants;

public class Elavator extends SubsystemBase {
  public enum Level {
    Home(0.0), Level_1(15.0), Level_2(30.0), Level_3(35.0), Level_4(40.0);


    private double pos;
    private Level(Double pos) {
      Preferences.initDouble("Elavator/Position/" + this.name(), pos);
      this.pos = Preferences.getDouble("Elavator/Position/" + this.name(), pos);
    }

    public double getposition(){
      return this.pos;
    }
  };

  private TalonFX elavationMotor = new TalonFX(CanID.ElevatorID);
  public enum ElevationControl {
    Home, Stopped, Zeroizing, Moving,
  };
  private ElevationControl targetState;
  private Level targetLevel;
  private double position;
  private StatusSignal<ReverseLimitValue> motorReverseLimit = elavationMotor.getReverseLimit();

  public void setLevel(Level level) {
    targetLevel = level;
    moveTo(targetLevel.getposition());
  }

  public void moveTo(double position) {
      this.position = position;
    
      elavationMotor.setNeutralMode(NeutralModeValue.Brake);
      switch (targetState) {
      case Home:
        targetState = ElevationControl.Zeroizing;
        elavationMotor.set(-0.01);
        break;
      case Zeroizing:
        break;
      case Stopped:
        targetState = ElevationControl.Moving;
        elavationMotor.setControl(new MotionMagicVoltage(position));
        break;
      case Moving:
        elavationMotor.setControl(new MotionMagicVoltage(position));
        break;
    }

  }

  public void stop(){
    elavationMotor.set(0);
  }

  public boolean isAtPosition(){
    return Math.abs(position - elavationMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.POSITIONACCURACY;
  }

  private void checkCurrentLimit(){
    if (elavationMotor.getTorqueCurrent().getValueAsDouble() > Constants.ElavationConstants.CURRENTLIMIT){
      elavationMotor.stopMotor();
      elavationMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }


  /** Creates a new Elavator. */
  public Elavator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkCurrentLimit();

    switch (targetState) {
      case Home:
        
        break;
      case Zeroizing:
        motorReverseLimit.refresh();
        if(motorReverseLimit.getValue() == ReverseLimitValue.Open){
          elavationMotor.setPosition(0);
          targetState = ElevationControl.Moving;
          elavationMotor.setControl(new MotionMagicVoltage(position));
        }

        if(targetLevel == Level.Home) {
          targetState = ElevationControl.Home;
        }
        break;
      case Stopped:

        break;
      case Moving:
        if (isAtPosition()){
          elavationMotor.stopMotor();
          targetState = ElevationControl.Stopped;
        }
        break;
    }
  }
}
