// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBus;
import frc.robot.lib.ConfigCheck;

public class Climber extends SubsystemBase {
  private TalonFX primary = new TalonFX(CanBus.ClimberPrimary);
  private MotionMagicDutyCycle magicSetPosition = new MotionMagicDutyCycle(0.0);


  /** Creates a new Climber. */
  public Climber() {
    ConfigCheck check = new ConfigCheck("Config/Climber/Primary", primary);
    Command chkCommand = new InstantCommand( () -> {
      check.SaveCheck();
    }).ignoringDisable(true);
    chkCommand.setName("Config/Climber/UpdatePrimary");
    SmartDashboard.putData(chkCommand);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    primary.set(0);
  }

  public void holdPosition() {
    double currentPosition = primary.getPosition().getValueAsDouble();
    primary.setControl(magicSetPosition.withPosition(currentPosition));
  }

  private double MAX_DISTANCE = 10.0;
  public void move(double distance) {
    distance = MAX_DISTANCE * MathUtil.clamp(distance, -1.0, 1.0);

    double pos = primary.getPosition().getValueAsDouble() + distance;
    primary.setControl(magicSetPosition.withPosition(pos));
  }

}
