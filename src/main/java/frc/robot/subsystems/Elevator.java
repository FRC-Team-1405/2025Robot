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
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
import frc.robot.lib.MotorSim.MotorSim_Mech;
import frc.robot.lib.MotorSim.PhysicsSim;


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

  private MotorSim_Mech motorSim_Mech = new MotorSim_Mech("ElevatorMechanism");

  public enum ElevationControl {
    Home, Stopped, Zeroizing, Moving,
  };
  private ElevationControl targetState = ElevationControl.Home;
  private ElevationLevel targetLevel = ElevationLevel.Home;
  private double position = targetLevel.getposition();
  private StatusSignal<ReverseLimitValue> motorReverseLimit = mainMotor.getReverseLimit();
  private Alert motorTorquewarning = new Alert("Elavator motor is using more power than permiter (possible stall)", AlertType.kWarning);
  private Mechanism2d mechanism = new Mechanism2d(3, 40);
  private MechanismRoot2d root = mechanism.getRoot("ElevatorRoot", 1.5, 0);
  private MechanismLigament2d elavatorLigament = new MechanismLigament2d("Elevator", 0, 90, 6, new Color8Bit(Color.kGray));
  private Map<ElevationLevel, MechanismLigament2d> levelIndicators = new HashMap<>();
  private Map<ElevationLevel, Color> levelBaseColors = new HashMap<>();
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
    mainMotor.set(0);
  }

  public void stopArm(){
    armMotor.set(0);
  }

  public void setArmlevel(ArmLevel level) {
    armMotor.setControl(new MotionMagicVoltage(level.getposition()));
  }

  public boolean isArmAtLevel(ArmLevel level) {
    if (Robot.isSimulation()){
      return true;
    }
    return Math.abs(level.getposition() - armMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.POSITIONACCURACY;
  }

  public boolean isAtPosition(){
    // if (Robot.isSimulation()){
    //   return true;
    // }
    
    return Math.abs(position - mainMotor.getPosition().getValue().in(Rotations)) < Constants.ElavationConstants.POSITIONACCURACY;
  }

  private void checkCurrentLimit(){
    if (!Utils.isSimulation() && Math.abs(mainMotor.getTorqueCurrent().getValueAsDouble()) > Constants.ElavationConstants.CURRENTLIMIT){
      mainMotor.stopMotor();
      mainMotor.setNeutralMode(NeutralModeValue.Coast);
      motorTorquewarning.set(true);
    }
  }

  public Elevator() {
    simulationInit();
    initElevatorMechanism();

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

     /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = mainMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

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

    System.out.println(String.format("targetState: %s, main motor position: %.1f", targetState, mainMotor.getPosition().getValue().in(Rotations)));
    motorSim_Mech.update(mainMotor.getPosition(), mainMotor.getVelocity());
    SmartDashboard.putNumber("Elevator/Position", getElevatorPos());
    SmartDashboard.putNumber("Elevator/Arm Position", getArmPosition());
  }

  // @Override
  // public void simulationPeriodic() {
  //   // Update the motor simulation state based on the main motor's control input
  //   mainMotorSim.setSupplyVoltage(mainMotor.getSimState().getMotorVoltage());
  //   slaveMotorSim.setInputVoltage(slaveMotor.getSimState().getMotorVoltage());

  //   // Simulate the motor voltage
  //   double motorVoltage = mainMotorSim.getMotorVoltage();

  //   // Update the elevator simulation
  //   elevatorSim.setInput(motorVoltage);
  //   elevatorSim.update(0.02); // 20ms simulation step

  //   // Update the simulated elevator position
  //   elevatorPosition = elevatorSim.getPositionMeters();

  //   // Simulate encoder position
  //   // Convert elevator position (meters) to motor rotor position (rotations)
  //   double rotorPositionRotations = elevatorPosition / (2 * Math.PI * kElevatorDrumRadius) * kElevatorGearRatio;
  //   mainMotorSim.setRawRotorPosition(rotorPositionRotations);
  //   slaveMotorSim.setRawRotorPosition(rotorPositionRotations);
  // }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(mainMotor, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  private MechanismLigament2d createLevelIndicator(String name, double position, Color color) {
    // Create an invisible/neutral vertical anchor to position the ticks at 'position' along the root
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
    // Add the elevator spine and the moving ligament
    // spine gives context for the ruler; elevator ligament shows current position
    // MechanismLigament2d spine = new MechanismLigament2d("Spine", 40.0, 90, 3, new Color8Bit(Color.kBlack));
    // root.append(spine);
    root.append(elavatorLigament);

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
  }

  private void updateElevatorMechanism() {
    // Update visual bar length from motor position (assumes motor position units map to level units)
    double motorPos = mainMotor.getPosition().getValue().in(Rotations);

    elavatorLigament.setLength(motorPos);

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
  }
}
