// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBus;
import frc.robot.Constants.CanID;
import frc.robot.Constants.DigitalIO;


public class Elevator extends SubsystemBase {
  public enum ElevationLevel {
    Home(0.0), Level_1(0.0), Level_2(6.5), Level_3(18.0), Level_4(36.5), Inverted_Low(12.6), Level_4_Auto(39.3);

    private double pos;
    private ElevationLevel(Double pos) {
      Preferences.initDouble("Elavator/Position/" + this.name(), pos);
      this.pos = Preferences.getDouble("Elavator/Position/" + this.name(), pos);
    }

    public double getposition(){
      return this.pos;
    }
  };

  public enum ArmLevel {
    Home(0.0), 
    Travel(3.5), 
    Low_Score(0.0), 
    Middle_Score(3.0), 
    High_Score(7.5),
    Max_Value(30.8),
    Inverted_Low(27.0),
    Climb(15.0),
    Algae(20.0),
    Algae_Output(20.0),
    High_Score_Auto(9.0);

    private double pos;
    private ArmLevel(Double pos) {
      // Preferences.initDouble("Arm/Position/" + this.name(), pos);
      // this.pos = Preferences.getDouble("Arm/Position/" + this.name(), pos);
      this.pos = pos;
    }

    public double getposition(){
      return this.pos;
    }
  };

  private TalonFX mainMotor = new TalonFX(CanBus.ElevatorPrimaryID);
  private TalonFX slaveMotor = new TalonFX(CanBus.ElevatorSecondaryID);
  private TalonFX armMotor = new TalonFX(CanBus.ArmMotorPrimaryID);
  private final DigitalInput forwardLimit = new DigitalInput(DigitalIO.ElevatorForwardLimit);
  private final DigitalInput reverseLimit = new DigitalInput(DigitalIO.ElevatorReverseLimit);
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

  public enum ElevationControl {
    Home, Stopped, Zeroizing, Moving,
  };
  private ElevationControl targetState = ElevationControl.Home;
  private ElevationLevel targetLevel = ElevationLevel.Home;
  private double position = targetLevel.getposition();
  private StatusSignal<ReverseLimitValue> motorReverseLimit = mainMotor.getReverseLimit();
  private Alert motorTorquewarning = new Alert("Elavator motor is using more power than permiter (possible stall)", AlertType.kWarning);
  private Mechanism2d mechanism = new Mechanism2d(3, 3);
  private MechanismLigament2d elavatorLigament;
  private MechanismLigament2d armMechanismLigament;

  public void setLevel(ElevationLevel level) {
    targetLevel = level;
    moveTo(targetLevel.getposition());

    if(targetLevel == ElevationLevel.Home) {
      targetState = ElevationControl.Home;
    }
  }


  public double getElevatorPos(){
      return mainMotor.getPosition().getValue().in(Rotations)/ElevationLevel.Level_4.getposition();
  }


  public double getArmPosition(){
   return armMotor.getPosition().getValue().in(Rotations);

  }

  public void moveTo(double position) {
      this.position = position;
    
      motorTorquewarning.set(false);
      mainMotor.setNeutralMode(NeutralModeValue.Brake);
      switch (targetState) {
      case Home:
        targetState = ElevationControl.Zeroizing;
        mainMotor.set(-0.1);
        break;
      case Zeroizing:
        break;
      case Stopped:
        targetState = ElevationControl.Moving;
        mainMotor.setControl(new MotionMagicVoltage(position));
        break;
      case Moving:
        mainMotor.setControl(new MotionMagicVoltage(position));
        break;
    }

  }

  public void stopElevator(){
    mainMotor.set(0);
  }

  public void stopArm(){
    armMotor.set(0);
  }

  public void setArmlevel(ArmLevel level) {
    armMotor.setControl(new MotionMagicVoltage(level.getposition()));
  }

  public boolean isArmAtLevel(ArmLevel level) {
    return Math.abs(level.getposition() - armMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.POSITIONACCURACY;
  }

  public boolean isAtPosition(){
    return Math.abs(position - mainMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.POSITIONACCURACY;
  }

  private void checkCurrentLimit(){
    if (Math.abs(mainMotor.getTorqueCurrent().getValueAsDouble()) > Constants.ElavationConstants.CURRENTLIMIT){
      mainMotor.stopMotor();
      mainMotor.setNeutralMode(NeutralModeValue.Coast);
      motorTorquewarning.set(true);
    }
  }

  public Elevator() {
    slaveMotor.setControl(new Follower(Constants.CanBus.ElevatorPrimaryID, false));

    MechanismRoot2d root = mechanism.getRoot("Root", 2, 0);
    elavatorLigament = root.append(new MechanismLigament2d("Elavator", 0, 90, 10, new Color8Bit(Color.kYellow)));
    armMechanismLigament = elavatorLigament.append( new MechanismLigament2d("Arm", 0.5, 90, 10, new Color8Bit(Color.kDarkGreen)));
    SmartDashboard.putData("Elavator/Mech2d", mechanism);
  }

  @Override
  public void periodic() {
    // mainMotor.setControl(dutyCycle.withOutput(0.5)
    //                               .withLimitForwardMotion(forwardLimit.get())
    //                               .withLimitReverseMotion(reverseLimit.get()));

     
    // This method will be called once per scheduler run
    checkCurrentLimit();

    switch (targetState) {
      case Home:
        
        break;
      case Zeroizing:
        // motorReverseLimit.refresh();
        // if(motorReverseLimit.getValue() == ReverseLimitValue.ClosedToGround){
          mainMotor.setPosition(0);
          targetState = ElevationControl.Moving;
          moveTo(position);
        // }
        break;
      case Stopped:

        break;
      case Moving:        
        if (isAtPosition()){
          targetState = ElevationControl.Stopped;
        }
        break;
    }

    SmartDashboard.putNumber("Elevator/Position", getElevatorPos());
    SmartDashboard.putNumber("Elevator/Arm Position", getArmPosition());
  }
}
