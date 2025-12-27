// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBus;
import frc.robot.Constants.DigitalIO;
import frc.robot.Robot;
import frc.robot.lib.FinneyLogger;
import frc.robot.lib.MotorSim.MotorSim_Mech;
import frc.robot.lib.MotorSim.PhysicsSim;


public class Elevator extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  public enum ElevationLevel {
    // Home(0.0), Level_1(0.0), Level_2(6.5), Level_3(18.0), Level_4(36.5), Inverted_Low(12.6), Level_4_Auto(39.3); 1:1 gear ratio

    // Gear ratio of 7.75
    Home(0.0),
    Level_1(0.0),
    Level_2(0.839),
    Level_3(2.323),
    Level_4(5.071),
    Inverted_Low(1.626),
    Level_4_Auto(5.071);


    private double pos;
    private ElevationLevel(Double pos) {
      // Preferences.initDouble("Elevator/Position/" + this.name(), pos);
      this.pos = pos;
    }

    public double getposition(){
      return this.pos;
    }
  };

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

  private TalonFX mainMotor = new TalonFX(CanBus.ElevatorPrimaryID);
  private TalonFX slaveMotor = new TalonFX(CanBus.ElevatorSecondaryID);
  // private final TalonFXSimState mainMotorSim = mainMotor.getSimState();
  // private final TalonFXSimState slaveMotorSim = slaveMotor.getSimState();

  private TalonFX armMotor = new TalonFX(CanBus.ArmMotorPrimaryID);
  private final DigitalInput forwardLimit = new DigitalInput(DigitalIO.ElevatorForwardLimit);
  private final DigitalInput reverseLimit = new DigitalInput(DigitalIO.ElevatorReverseLimit);
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

  // private static final double kElevatorGearRatio = 10.0; // Gear ratio
  // private static final double kElevatorDrumRadius = Units.inchesToMeters(1.0); // Drum radius in meters
  // private static final double kCarriageMass = 5.0; // Mass of the elevator carriage in kg

  // Create a DC motor model for the elevator
  // private final DCMotor elevatorMotorModel = DCMotor.getKrakenX60(2);

  // Create a linear system simulation for the elevator
//   private static final LinearSystem<N1, N1, N1> elevatorSystem = ElevatorSystem.createElevatorSystem(
//     DCMotor.getFalcon500(2), // Two Falcon 500 motors
//     kElevatorGearRatio,
//     kElevatorDrumRadius,
//     kElevatorMass
// );

  // private final ElevatorSim elevatorSim = new ElevatorSim(
  //     DCMotor.getFalcon500(2), // Two Falcon 500 motors
  //     kElevatorGearRatio,      // Gear ratio
  //     kCarriageMass,          // Mass of the elevator carriage in kg
  //     kElevatorDrumRadius,     // Drum radius in meters
  //     0,  // Min height in meters
  //     1,  // Max height in meters
  //     true,  // Simulate gravity
  //     0.0                     // Minimum height in meters
  // );

  // private double elevatorPosition = 0.0; // Simulated elevator position in meters

  private MotorSim_Mech elevator_motorSimMech = new MotorSim_Mech("ElevatorMotorSimMech");
  private MotorSim_Mech arm_motorSimMech = new MotorSim_Mech("ArmMotorSimMech");

  public enum ElevationControl {
    Home, Stopped, Zeroizing, Moving,
  };
  private ElevationControl targetState = ElevationControl.Home;
  private ElevationLevel targetLevel = ElevationLevel.Home;
  private double position = targetLevel.getposition();
  private StatusSignal<ReverseLimitValue> motorReverseLimit = mainMotor.getReverseLimit();
  private Alert motorTorquewarning = new Alert("Elevator motor is using more power than permiter (possible stall)", AlertType.kWarning);
  private Mechanism2d mechanism = new Mechanism2d(3, ElevationLevel.Level_4_Auto.getposition() + ROOT_Y_OFFSET);
  private static final double ROOT_Y_OFFSET = 4.0;
  private MechanismRoot2d root = mechanism.getRoot("ElevatorRoot", 1.5, ROOT_Y_OFFSET);
  private MechanismLigament2d elevatorLigament;
  private Map<ElevationLevel, MechanismLigament2d> levelIndicators = new HashMap<>();
  private Map<ElevationLevel, Color> levelBaseColors = new HashMap<>();
  private MechanismLigament2d armMechanismLigament;

  public void setLevel(ElevationLevel level) {
    targetLevel = level;
    moveTo(targetLevel.getposition());

    fLogger.log("setLevel Elevator called with level: %s (%.1f), variables -- position: %.2f, targetLevel: %s", level.name(), level.getposition(), position, targetLevel.name());

    if(targetLevel == ElevationLevel.Home) {
      targetState = ElevationControl.Home;
    }
  }


  public double getElevatorPos(){
      return mainMotor.getPosition().getValue().in(Rotations);
  }


  public double getArmPosition(){
   return armMotor.getPosition().getValue().in(Rotations);
  }

  public void moveTo(double position) {
    this.position = position;
  
    motorTorquewarning.set(false);
    mainMotor.setNeutralMode(NeutralModeValue.Brake);
    slaveMotor.setNeutralMode(NeutralModeValue.Brake);
    mainMotor.setControl(new MotionMagicVoltage(position));
    //   switch (targetState) {
    //   case Home:
    //     targetState = ElevationControl.Zeroizing;
    //     mainMotor.set(-0.1);
    //     break;
    //   case Zeroizing:
    //     break;
    //   case Stopped:
    //     targetState = ElevationControl.Moving;
    //     mainMotor.setControl(new MotionMagicVoltage(position));
    //     break;
    //   case Moving:
    //     mainMotor.setControl(new MotionMagicVoltage(position));
    //     break;
    // }

  }

  public void stopElevator(){
    fLogger.log("Stop Elevator called");

    // hold current elevator position, set(0) or stopMotor() doesn't hold position
    // mainMotor.setControl(new PositionVoltage(mainMotor.getPosition().getValue()));
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

  public boolean isAtPosition(double position){
    boolean isWithinTolerance = Math.abs(position - mainMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.ELEVATOR_POSITION_ACCURACY;
    boolean isStopped = Math.abs(mainMotor.getVelocity().getValue().in(RotationsPerSecond)) < 0.1;
    // boolean isGreaterThanPosition = mainMotor.getPosition().getValue().in(Rotations) > position; // Prevents overshoot on downward movements

    fLogger.log("isWithinTolerance: %s (%.1f), isStopped: %s (%.1f)", isWithinTolerance, Math.abs(position - mainMotor.getPosition().getValue().in(Rotations)), isStopped, Math.abs(mainMotor.getVelocity().getValue().in(RotationsPerSecond)));
    fLogger.log("elevator target position: %.2f, current position: %.2f", position, mainMotor.getPosition().getValue().in(Rotations));

    return isWithinTolerance && isStopped;
  }

  public boolean isAtLevel(ElevationLevel level) {
    return isAtPosition(level.getposition());
  }

  private void checkCurrentLimit(){
    if (!Utils.isSimulation() && Math.abs(mainMotor.getTorqueCurrent().getValueAsDouble()) > Constants.ElavationConstants.CURRENTLIMIT){
      System.err.println("ALERT: ELEVATOR MOTOR CURRENT LIMIT EXCEEDED");
      mainMotor.stopMotor();
      mainMotor.setNeutralMode(NeutralModeValue.Coast);
      motorTorquewarning.set(true);
    }
  }

  public Elevator() {
    setupMotors();
    simulationInit();
    initElevatorMechanism();
  }

  private void setupMotors() {
    //
    // Elevator Motor Configuration
    //

    TalonFXConfiguration elevator_cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs elevator_fdb = elevator_cfg.Feedback;
    elevator_fdb.SensorToMechanismRatio = 7.75; // x rotor rotations per mechanism rotation

     /* Configure Motion Magic */
    MotionMagicConfigs elevator_mm = elevator_cfg.MotionMagic;
    elevator_mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10)) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15)); // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      // .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    Slot0Configs elevator_slot0 = elevator_cfg.Slot0;
    elevator_slot0.kS = 0;
    elevator_slot0.kV = 0.0;
    elevator_slot0.kA = 0.0;
    elevator_slot0.kP = 6;
    elevator_slot0.kI = 0;
    elevator_slot0.kD = 0.0;
    elevator_slot0.kG = 0;

    CurrentLimitsConfigs limits = elevator_cfg.CurrentLimits;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = 30;   // continuous supply limit
    limits.StatorCurrentLimitEnable = true;
    limits.StatorCurrentLimit = 40;   // continuous stator limit

    SoftwareLimitSwitchConfigs soft = elevator_cfg.SoftwareLimitSwitch;
    soft.ForwardSoftLimitEnable = true;
    soft.ForwardSoftLimitThreshold = 5.1;  // mechanism rotations (after gear ratio)
    soft.ReverseSoftLimitEnable = true;
    soft.ReverseSoftLimitThreshold = 0.0;


    StatusCode elevator_status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      elevator_status = mainMotor.getConfigurator().apply(elevator_cfg);
      if (elevator_status.isOK()) break;
    }
    if (!elevator_status.isOK()) {
      System.out.println("Could not configure Elevator mainMotor. Error: " + elevator_status.toString());
    }

    StatusCode elevator_status_slave = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      elevator_status_slave = slaveMotor.getConfigurator().apply(elevator_cfg);
      if (elevator_status_slave.isOK()) break;
    }
    if (!elevator_status_slave.isOK()) {
      System.out.println("Could not configure Elevator slaveMotor. Error: " + elevator_status_slave.toString());
    }

    mainMotor.setNeutralMode(NeutralModeValue.Brake);
    slaveMotor.setNeutralMode(NeutralModeValue.Brake);

    slaveMotor.setControl(new Follower(Constants.CanBus.ElevatorPrimaryID, false));

    mainMotor.setPosition(0);

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
    // mainMotor.setControl(dutyCycle.withOutput(0.5)
    //                               .withLimitForwardMotion(forwardLimit.get())
    //                               .withLimitReverseMotion(reverseLimit.get()));

     
    // This method will be called once per scheduler run
    // checkCurrentLimit();

    // switch (targetState) {
    //   case Home:
        
    //     break;
    //   case Zeroizing:
    //     // motorReverseLimit.refresh();
    //     // if(motorReverseLimit.getValue() == ReverseLimitValue.ClosedToGround){
    //       mainMotor.setPosition(0);
    //       targetState = ElevationControl.Moving;
    //       moveTo(position);
    //     // }
    //     break;
    //   case Stopped:

    //     break;
    //   case Moving:        
    //     if (isAtPosition()){
    //       targetState = ElevationControl.Stopped;
    //     }
    //     break;
    // }

    updateElevatorMechanism();

    // System.out.println(String.format("Elevator position: %.2f, velocity: %.2f", mainMotor.getPosition().getValue().in(Rotations), Math.abs(mainMotor.getVelocity().getValue().in(RotationsPerSecond))));
    System.out.println(String.format("Arm position: %.3f, velocity: %.2f", armMotor.getPosition().getValue().in(Rotations), Math.abs(armMotor.getVelocity().getValue().in(RotationsPerSecond))));
    elevator_motorSimMech.update(mainMotor.getPosition(), mainMotor.getVelocity());
    arm_motorSimMech.update(armMotor.getPosition(), armMotor.getVelocity());
    SmartDashboard.putNumber("Elevator/Position", getElevatorPos());
    SmartDashboard.putNumber("Elevator/Arm Position", getArmPosition());
    SmartDashboard.putNumber("Elevator/Velocity", mainMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Arm Velocity", armMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Acceleration", mainMotor.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Arm Acceleration", armMotor.getAcceleration().getValueAsDouble());
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(mainMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  private MechanismLigament2d createLevelIndicator(String name, double position, Color color) {
    // Create an invisible/neutral vertical anchor to position the ticks at 'position' along the root
    // offset anchor by ROOT_Y_OFFSET so ticks line up with the visualized ligament (which is shifted)
    MechanismLigament2d anchor = new MechanismLigament2d(name + "_anchor", position, 90, 0.1, new Color8Bit(Color.kGray));
    root.append(anchor);
    // small ticks left/right to look like ruler markings
    MechanismLigament2d tickR = new MechanismLigament2d(name + "_tickR", 0.6, 0, 3, new Color8Bit(color));
    anchor.append(tickR);
    MechanismLigament2d tickL = new MechanismLigament2d(name + "_tickL", 0.6, 180, 3, new Color8Bit(color));
    anchor.append(tickL);
    // return one of the visible tick ligaments so we can change its color later
    return tickR;
  }

  private void initElevatorMechanism() {
    // Use the class-level root (already created with ROOT_Y_OFFSET).
    // Initialize elevator ligament so 0 maps to ROOT_Y_OFFSET and allow negative overshoot to be visible.
    elevatorLigament = root.append(new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit(Color.kYellow)));
    // Create arm ligament attached to the elevator ligament. Angle maps to ArmLevel values (assumed degrees).
    armMechanismLigament = elevatorLigament.append(new MechanismLigament2d("Arm", 1.5, (float)ArmLevel.Home.getposition() + 90, 10, new Color8Bit(Color.kDarkGreen)));

    // Add the elevator spine and the moving ligament
    // spine gives context for the ruler; elevator ligament shows current position
    // MechanismLigament2d spine = new MechanismLigament2d("Spine", 40.0, 90, 3, new Color8Bit(Color.kBlack));
    // root.append(spine);

    // create level indicators (ticks) with distinct colors
    levelIndicators.put(ElevationLevel.Home, createLevelIndicator("Home", ElevationLevel.Home.getposition(), Color.kGreen));
    levelBaseColors.put(ElevationLevel.Home, Color.kGreen);
    levelIndicators.put(ElevationLevel.Level_1, createLevelIndicator("Level_1", ElevationLevel.Level_1.getposition(), Color.kBlue));
    levelBaseColors.put(ElevationLevel.Level_1, Color.kBlue);
    levelIndicators.put(ElevationLevel.Level_2, createLevelIndicator("Level_2", ElevationLevel.Level_2.getposition(), Color.kYellow));
    levelBaseColors.put(ElevationLevel.Level_2, Color.kYellow);
    levelIndicators.put(ElevationLevel.Level_3, createLevelIndicator("Level_3", ElevationLevel.Level_3.getposition(), Color.kOrange));
    levelBaseColors.put(ElevationLevel.Level_3, Color.kOrange);
    levelIndicators.put(ElevationLevel.Level_4, createLevelIndicator("Level_4", ElevationLevel.Level_4.getposition(), Color.kRed));
    levelBaseColors.put(ElevationLevel.Level_4, Color.kRed);
    levelIndicators.put(ElevationLevel.Inverted_Low, createLevelIndicator("Inverted_Low", ElevationLevel.Inverted_Low.getposition(), Color.kPurple));
    levelBaseColors.put(ElevationLevel.Inverted_Low, Color.kPurple);
    levelIndicators.put(ElevationLevel.Level_4_Auto, createLevelIndicator("Level_4_Auto", ElevationLevel.Level_4_Auto.getposition(), Color.kCyan));
    levelBaseColors.put(ElevationLevel.Level_4_Auto, Color.kCyan);

    SmartDashboard.putData("Elevator/Mech2d", mechanism);
  }

  private void updateElevatorMechanism() {
    // Update visual bar length from motor position (assumes motor position units map to level units)
    double motorPos = mainMotor.getPosition().getValue().in(Rotations);

    elevatorLigament.setLength(motorPos);

    // update indicators color when passed the current length
    for (Map.Entry<ElevationLevel, MechanismLigament2d> e : levelIndicators.entrySet()) {
      ElevationLevel lvl = e.getKey();
      MechanismLigament2d tick = e.getValue();
      Color base = levelBaseColors.getOrDefault(lvl, Color.kWhite);
      if (lvl.getposition() <= motorPos) {
        tick.setColor(new Color8Bit(Color.kGreen));
      } else {
        tick.setColor(new Color8Bit(base));
      }
    }
    
    // Update arm visual: map arm position/level to ligament angle.
    // Prefer using the real motor position (in degrees) if available; otherwise use configured levels.
    if (armMechanismLigament != null) {
      double armAngleDeg = (armMotor.getPosition().getValue().in(Rotations) / ArmLevel.Max_Value.getposition()) + 90;
      // If motor position is zero/invalid in simulation, fall back to configured level value
      // if (Double.isNaN(armAngleDeg) || Math.abs(armAngleDeg) < 1e-6) {
      //   armAngleDeg = ArmLevel.Home.getposition() + 90;
      // }
      // If your ArmLevel values are already degrees, you can directly set the angle.
      armMechanismLigament.setAngle(armAngleDeg);

      if (Robot.m_robotContainer.intake.hasCoral()){
        armMechanismLigament.setColor(new Color8Bit(Color.kWhite));
      } else {
        armMechanismLigament.setColor(new Color8Bit(Color.kDarkGreen));
      }
    }
  }
}
