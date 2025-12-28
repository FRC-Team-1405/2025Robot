// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBus;
import frc.robot.lib.FinneyLogger;
import frc.robot.lib.MotorSim.MotorSim_Mech;
import frc.robot.lib.MotorSim.PhysicsSim;


public class Arm extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  public enum ArmLevel {
    // gear ratio of 1:1
    // Home(0.0), 
    // Travel(3.5), 
    // Low_Score(0.0), 
    // Middle_Score(3.0), 
    // High_Score(7.5),
    // Max_Value(30.8),
    // Inverted_Low(27.0),
    // Climb(15.0),
    // Algae(20.0),
    // Algae_Output(20.0),
    // High_Score_Auto(9.0);

    // Gear ratio of 61.2
    Home(0.0),
    Travel(0.0341),
    Low_Score(0.0),
    Middle_Score(0.0292),
    High_Score(0.0877),
    Max_Value(0.3000),
    Inverted_Low(0.2630),
    Climb(0.1461),
    Algae(0.1948),
    Algae_Output(0.1948),
    High_Score_Auto(0.0877);



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

  private TalonFX armMotor = new TalonFX(CanBus.ArmMotorPrimaryID);

  private MotorSim_Mech arm_motorSimMech = new MotorSim_Mech("ArmMotorSimMech");

  // keep track of the arm position for the edification of the elevator's mechanism2d visualization. this isn't used for control.
  private static double armPosition = 0.0;

  public double getArmPosition(){
   return armMotor.getPosition().getValue().in(Rotations);
  }

  public void stopArm(){
    // hold current arm position, set(0) or stopMotor() doesn't hold position
    armMotor.setControl(new PositionVoltage(armMotor.getPosition().getValue()));
  }

  public void setArmlevel(ArmLevel level) {
    armMotor.setControl(new MotionMagicVoltage(level.getposition()));
  }

  public boolean isArmAtLevel(ArmLevel level) {
    boolean isWithinTolerance = Math.abs(level.getposition() - armMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.ARM_POSITION_ACCURACY;
    boolean isStopped = Math.abs(armMotor.getVelocity().getValue().in(RotationsPerSecond)) < 0.1;
    return isWithinTolerance && isStopped;
  }

  /**
   * Arm is at or past travel position and is safe to move elevator.
   * @return
   */
  public boolean isArmSafeToTravel() {
    boolean minThresholdSafe = armMotor.getPosition().getValue().in(Rotations) > (ArmLevel.Travel.getposition()-Constants.ElavationConstants.ARM_POSITION_ACCURACY);
    boolean maxThresholdSafe = armMotor.getPosition().getValue().in(Rotations) < (ArmLevel.High_Score_Auto.getposition()+Constants.ElavationConstants.ARM_POSITION_ACCURACY);
    return minThresholdSafe && maxThresholdSafe;
  }

  public Arm() {
    setupMotors();
    simulationInit();
    // initElevatorMechanism();
  }

  private void setupMotors() {
    //
    // Arm Motor Configuration
    //

    TalonFXConfiguration arm_cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs arm_fdb = arm_cfg.Feedback;
    arm_fdb.SensorToMechanismRatio = 61.2; // x rotor rotations per mechanism rotation

     /* Configure Motion Magic */
    MotionMagicConfigs arm_mm = arm_cfg.MotionMagic;
    arm_mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5));
      // .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    Slot0Configs arm_slot0 = arm_cfg.Slot0;
    arm_slot0.kS = 0;
    arm_slot0.kV = 0.0;
    arm_slot0.kA = 0.0;
    arm_slot0.kP = 100;
    arm_slot0.kI = 0;
    arm_slot0.kD = 0;

    StatusCode arm_status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      arm_status = armMotor.getConfigurator().apply(arm_cfg);
      if (arm_status.isOK()) break;
    }
    if (!arm_status.isOK()) {
      System.out.println("Could not configure Arm. Error: " + arm_status.toString());
    }

    armMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    armPosition = armMotor.getPosition().getValue().in(Rotations);
    // updateElevatorMechanism();

    // System.out.println(String.format("Arm position: %.3f, velocity: %.2f", armMotor.getPosition().getValue().in(Rotations), Math.abs(armMotor.getVelocity().getValue().in(RotationsPerSecond))));
    arm_motorSimMech.update(armMotor.getPosition(), armMotor.getVelocity());
    SmartDashboard.putNumber("Elevator/Arm Position", getArmPosition());
    SmartDashboard.putNumber("Elevator/Arm Velocity", armMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Arm Acceleration", armMotor.getAcceleration().getValueAsDouble());
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public static double getArmPositionForElevator() {
    return armPosition;
  }
}
