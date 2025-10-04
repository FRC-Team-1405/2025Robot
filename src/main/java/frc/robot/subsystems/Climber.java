// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.rmi.server.ServerCloneException;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBus;
import frc.robot.lib.ConfigCheck;

public class Climber extends SubsystemBase {
  private TalonFX primary = new TalonFX(CanBus.ClimberPrimary);
  private TalonFX secondary = new TalonFX(CanBus.ClimberSecondary);
  private MotionMagicVoltage magicSetPosition = new MotionMagicVoltage(0.0);
  private Servo intakeServo = new Servo(9);
  private double servoPos = 1.0;


  /** Creates a new Climber. */
  public Climber() {
    secondary.setControl(new Follower(Constants.CanBus.ClimberPrimary, true));
    {
      ConfigCheck check = new ConfigCheck("Config/Climber/Primary", primary);
      Command chkCommand = new InstantCommand( () -> {
        check.SaveCheck();
      }).ignoringDisable(true);
      chkCommand.setName("Config/Climber/UpdatePrimary");
      SmartDashboard.putData(chkCommand);
    }
    {
      ConfigCheck check = new ConfigCheck("Config/Climber/Secondary", secondary);
      Command chkCommand = new InstantCommand( () -> {
        check.SaveCheck();
      }).ignoringDisable(true);
      chkCommand.setName("Config/Climber/UpdateSecondary");
      SmartDashboard.putData(chkCommand);
    }
    if (SmartDashboard.containsKey("intakeServo") == false) {
      SmartDashboard.putNumber("intakeServo", 0.0);
    }
    intakeServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    Command closeServo = this.runOnce( () -> { servoPos += 0.1 ; setServoPos(servoPos); });
    closeServo.setName("Close Servo");
    Command openServo  = this.runOnce( () -> { servoPos -= 0.1 ; setServoPos(servoPos); });
    openServo.setName("Open Sero");
    SmartDashboard.putData(closeServo);
    SmartDashboard.putData(openServo);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climber/Upper Limit", primary.getFault_ForwardSoftLimit().getValue());
    SmartDashboard.putBoolean("Climber/Lower Limit", primary.getFault_ReverseSoftLimit().getValue());
    // This method will be called once per scheduler run
  }

  public void stop() {
    primary.set(0);
  }

  public void zeroize() {
    primary.setPosition(0);
  }
  
  private double MAX_DISTANCE = 5.0;
  public void move(double distance) {
    distance = MAX_DISTANCE * MathUtil.clamp(distance, -1.0, 1.0);

    double pos = primary.getPosition().getValueAsDouble() + distance;
    primary.setControl(magicSetPosition.withPosition(pos));
  }

  public void setServoPos(double pos) {
    pos = MathUtil.clamp(pos, 0.0, 1.0);
    intakeServo.set(pos);
    servoPos = pos; 
  }

}
