// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Tracer;
import frc.robot.lib.TagMapper.TagMapper;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private TagMapper mapper;


  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    m_robotContainer = new RobotContainer();
  }

  @Override
  protected void loopFunc() {
    Tracer.traceFunc("RobotLoop", super::loopFunc);
  }

  @Override
  public void robotInit() {
    DataLogManager.start();
    mapper = new TagMapper("Left");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.correctOdometry();

    mapper.update();
    mapper.publishTagMap();
    // SmartDashboard.putData("TagField", mapper.getField());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    mapper.updateCurrentPose();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
