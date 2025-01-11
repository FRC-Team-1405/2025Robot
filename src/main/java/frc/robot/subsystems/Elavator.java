// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.Preferences;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elavator extends SubsystemBase {
  private TalonFX elavationMotor;
  private enum Level {
    home(0.0), l1(15.0), l2(30.0), l3(35.0), l4(40.0);


    private double pos;
    private Level(Double pos) {
      Preferences.initDouble("Elavator/Position/" + this.name(), pos);
      this.pos = Preferences.getDouble("Elavator/Position/" + this.name(), pos);
    }

    public double getposition(){
      return this.pos;
    }
  };
  private Level targetLevel;

  public Level getlevel(){
    return targetLevel;
  }

  public void setlevel(Level level){
    targetLevel = level;
    elavationMotor.setPosition(level.getposition());
  }

  public void stop(){
    elavationMotor.set(0);
  }

  public boolean isAtPosition(){
    return Math.abs(targetLevel.getposition() - elavationMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.POSITIONACCURACY;
  }


  /** Creates a new Elavator. */
  public Elavator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
