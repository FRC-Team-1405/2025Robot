// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
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

  private TalonFX mainMotor = new TalonFX(CanBus.ElevatorPrimaryID);
  private TalonFX slaveMotor = new TalonFX(CanBus.ElevatorSecondaryID);
  private TalonFX armMotor = new TalonFX(CanBus.ArmMotorPrimaryID);
  public enum ElevationControl {
    Home, Stopped, Zeroizing, Moving,
  };
  private ElevationControl targetState = ElevationControl.Home;
  private Level targetLevel = Level.Home;
  private double position = targetLevel.getposition();
  private StatusSignal<ReverseLimitValue> motorReverseLimit = mainMotor.getReverseLimit();
  private Alert motorTorquewarning = new Alert("Elavator motor is using more power than permiter (possible stall)", AlertType.kWarning);
  private Mechanism2d mechanism = new Mechanism2d(3, 3);
  private MechanismLigament2d elavatorLigament;
  private MechanismLigament2d armMechanismLigament;

  public void setLevel(Level level) {
    targetLevel = level;
    moveTo(targetLevel.getposition());

    if(targetLevel == Level.Home) {
      targetState = ElevationControl.Home;
    }
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

  public void stop(){
    mainMotor.set(0);
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


  /** Creates a new Elavator. */
  public Elavator() {
    slaveMotor.setControl(new Follower(Constants.CanBus.ElevatorPrimaryID, true));

    MechanismRoot2d root = mechanism.getRoot("Root", 2, 0);
    elavatorLigament = root.append(new MechanismLigament2d("Elavator", 0, 90, 10, new Color8Bit(Color.kYellow)));
    armMechanismLigament = elavatorLigament.append( new MechanismLigament2d("Arm", 0.5, 90, 10, new Color8Bit(Color.kDarkGreen)));
    SmartDashboard.putData("Elavator/Mech2d", mechanism);
  }

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    checkCurrentLimit();

    switch (targetState) {
      case Home:
        
        break;
      case Zeroizing:
        motorReverseLimit.refresh();
        if(motorReverseLimit.getValue() == ReverseLimitValue.ClosedToGround){
          mainMotor.setPosition(0);
          targetState = ElevationControl.Moving;
          moveTo(position);
        }
        break;
      case Stopped:

        break;
      case Moving:
        moveArm();
        
        if (isAtPosition()){
          targetState = ElevationControl.Stopped;
        }
        break;
    }

    // ToDo convert to a min / max range and get encoder values
    elavatorLigament.setLength(3*MathUtil.inverseInterpolate(0, 1000, 250));
    armMechanismLigament.setAngle(90);
  }

  private void moveArm() {
        // get position
        double pos = mainMotor.getPosition().getValue().in(Rotations);

        // convert it
        pos = CalculateArmPosition(pos);
        // set arm pos
        armMotor.setControl(new PositionVoltage(pos));

  }
  private double CalculateArmPosition(double elevatorPosition) {
    // if < 25 then 0
    if(elevatorPosition < 25){
      return 0;
    } 

    if(elevatorPosition > 50){
      return 100;
    }

    return (elevatorPosition - 25.0) * 4.0;
  }
}
