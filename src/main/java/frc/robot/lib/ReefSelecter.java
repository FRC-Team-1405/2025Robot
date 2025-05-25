// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class ReefSelecter {
    public enum CoralLevel {
        Level_1,
        Level_2,
        Level_3,
        Level_4,
    };

    public enum Coral {
        Position_1,
        Position_2,
        Position_3,
        Position_4,
        Position_5,
        Position_6,
        Position_7,
        Position_8,
        Position_9,
        Position_10,
        Position_11,
        Position_12,
    }

    public enum Direction {
        Left,
        Right,
    }

    private Elevator.ElevationLevel level = Elevator.ElevationLevel.Level_4;
    private CoralLevel coralLevel = CoralLevel.Level_4;
    private Coral coralSelected = Coral.Position_1;

    StructPublisher<Pose2d> selectedReefPositionPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SelectedReefPosition", Pose2d.struct).publish();

    public ReefSelecter(){
        updatePosition();
    }
    public void setLevel(Elevator.ElevationLevel level){
        this.level = level;
        updatePosition();
    }

    public Elevator.ElevationLevel getLevel() {
        return level;
    }

    public void levelUp() {
        level = switch(level) {
            case Level_1 -> Elevator.ElevationLevel.Level_2;
            case Level_2 -> Elevator.ElevationLevel.Level_3;
            case Level_3 -> Elevator.ElevationLevel.Level_4;
            default -> level;
        };
        updatePosition();
    }
    
    public void levelDown(){
        level = switch(level){
            case Level_4 -> Elevator.ElevationLevel.Level_3;
            case Level_3 -> Elevator.ElevationLevel.Level_2;
            case Level_2 -> Elevator.ElevationLevel.Level_1;
            default -> level;
        };
        updatePosition();
    }

    public void selectLeft (){
        this.coralSelected = switch(this.coralSelected){
            case Position_1  -> Coral.Position_12;
            case Position_2  -> Coral.Position_1;
            case Position_3  -> Coral.Position_2;
            case Position_4  -> Coral.Position_3;
            case Position_5  -> Coral.Position_4;
            case Position_6  -> Coral.Position_5;
            case Position_7  -> Coral.Position_6;
            case Position_8  -> Coral.Position_7;
            case Position_9  -> Coral.Position_8;
            case Position_10 -> Coral.Position_9;
            case Position_11 -> Coral.Position_10;
            case Position_12 -> Coral.Position_11;
            default -> this.coralSelected;
        };
        updatePosition();
    }

    public void selectRight(){
        this.coralSelected = switch(this.coralSelected){
            case Position_1  -> Coral.Position_2;
            case Position_2  -> Coral.Position_3;
            case Position_3  -> Coral.Position_4;
            case Position_4  -> Coral.Position_5;
            case Position_5  -> Coral.Position_6;
            case Position_6  -> Coral.Position_7;
            case Position_7  -> Coral.Position_8;
            case Position_8  -> Coral.Position_9;
            case Position_9  -> Coral.Position_10;
            case Position_10 -> Coral.Position_11;
            case Position_11 -> Coral.Position_12;
            case Position_12 -> Coral.Position_1;
            default -> this.coralSelected;
        };
        updatePosition();
    }

    public void setCoralPosition(Coral position){
        this.coralSelected = position;
    }

    public Coral getCoralPosition(){
        return this.coralSelected;
    }

    public void updatePosition() {
        switch (level) {
            case Level_1:
                coralLevel = CoralLevel.Level_1;
                break;
            case Level_2:
                coralLevel = CoralLevel.Level_2;
                break;
            case Level_3:
                coralLevel = CoralLevel.Level_3;
                break;
            case Level_4:
                coralLevel = CoralLevel.Level_4;
                break;
            default:
                coralLevel = CoralLevel.Level_1;
                break;            
        }

        // update shuffleboard
        SmartDashboard.putBoolean("Reef/L1", coralLevel == CoralLevel.Level_1);
        SmartDashboard.putBoolean("Reef/L2", coralLevel == CoralLevel.Level_2);
        SmartDashboard.putBoolean("Reef/L3", coralLevel == CoralLevel.Level_3);
        SmartDashboard.putBoolean("Reef/L4", coralLevel == CoralLevel.Level_4);
        SmartDashboard.putBoolean("Reef/P1", coralSelected == Coral.Position_1);
        SmartDashboard.putBoolean("Reef/P2", coralSelected == Coral.Position_2);
        SmartDashboard.putBoolean("Reef/P3", coralSelected == Coral.Position_3);
        SmartDashboard.putBoolean("Reef/P4", coralSelected == Coral.Position_4);
        SmartDashboard.putBoolean("Reef/P5", coralSelected == Coral.Position_5);
        SmartDashboard.putBoolean("Reef/P6", coralSelected == Coral.Position_6);
        SmartDashboard.putBoolean("Reef/P7", coralSelected == Coral.Position_7);
        SmartDashboard.putBoolean("Reef/P8", coralSelected == Coral.Position_8);
        SmartDashboard.putBoolean("Reef/P9", coralSelected == Coral.Position_9);
        SmartDashboard.putBoolean("Reef/P10", coralSelected == Coral.Position_10);
        SmartDashboard.putBoolean("Reef/P11", coralSelected == Coral.Position_11);
        SmartDashboard.putBoolean("Reef/P12", coralSelected == Coral.Position_12);

        updateSelectedReefPositionVisualizer();
    }

    public void updateSelectedReefPositionVisualizer() {
        // if (RobotContainer.VISUALIZE_REEF_SELECTER_POSITION) {
            // TODO make this update a variable with the result, then add a getter for that result.
            // then use that getter when the driver presses B instead of getting the mapped position on the fly. slightly more efficient.
            getRobotPositionForCoral(getCoralPosition()).ifPresent(selectedPosition -> selectedReefPositionPublisher.set(selectedPosition));
        // }
    }

    /**
     * Process for position values was loosly documented in docs/MappingCoralPositionsToPValuesForReefSelector.md
     * @param coralToGetRobotPositionFor
     * @param currentAlliance
     * @return
     */
    public Optional<Pose2d> getRobotPositionForCoral(Coral coralToGetRobotPositionFor) {
        System.out.println("called getRobotPositionForCoral");
        if (DriverStation.getAlliance().isEmpty()){
            return Optional.empty();
        }

        Alliance currentAlliance = DriverStation.getAlliance().get();

        Pose2d selectedPosition;

        if (Alliance.Red.equals(currentAlliance)) {
            selectedPosition = switch(this.coralSelected) {
                case Position_1  -> new Pose2d(new Translation2d(11.78, 4.19), new Rotation2d(Units.degreesToRadians(180.00)));
                case Position_2  -> new Pose2d(new Translation2d(12.28, 5.05), new Rotation2d(Units.degreesToRadians(120.00)));
                case Position_3  -> new Pose2d(new Translation2d(12.56, 5.21), new Rotation2d(Units.degreesToRadians(120.00)));
                case Position_4  -> new Pose2d(new Translation2d(13.55, 5.21), new Rotation2d(Units.degreesToRadians(60.00)));
                case Position_5  -> new Pose2d(new Translation2d(13.84, 5.05), new Rotation2d(Units.degreesToRadians(60.00)));
                case Position_6  -> new Pose2d(new Translation2d(14.33, 4.19), new Rotation2d(Units.degreesToRadians(0.00)));
                case Position_7  -> new Pose2d(new Translation2d(14.33, 3.86), new Rotation2d(Units.degreesToRadians(0.00)));
                case Position_8  -> new Pose2d(new Translation2d(13.84, 3.00), new Rotation2d(Units.degreesToRadians( -60.00)));
                case Position_9  -> new Pose2d(new Translation2d(13.55, 2.84), new Rotation2d(Units.degreesToRadians( -60.00)));
                case Position_10 -> new Pose2d(new Translation2d(12.56, 2.84), new Rotation2d(Units.degreesToRadians( -120.00)));
                case Position_11 -> new Pose2d(new Translation2d(12.28, 3.00), new Rotation2d(Units.degreesToRadians( -120.00)));
                case Position_12 -> new Pose2d(new Translation2d(11.78, 3.86), new Rotation2d(Units.degreesToRadians(180.00)));
                default -> null;
            };
        } else {
            selectedPosition = switch(this.coralSelected) {
                case Position_1  -> new Pose2d(new Translation2d(5.77, 3.86), new Rotation2d(Units.degreesToRadians(0.00)));
                case Position_2  -> new Pose2d(new Translation2d(5.27, 3.00), new Rotation2d(Units.degreesToRadians( -60.00)));
                case Position_3  -> new Pose2d(new Translation2d(4.98, 2.84), new Rotation2d(Units.degreesToRadians( -60.00)));
                case Position_4  -> new Pose2d(new Translation2d(3.99, 2.84), new Rotation2d(Units.degreesToRadians( -120.00)));
                case Position_5  -> new Pose2d(new Translation2d(3.71, 3.00), new Rotation2d(Units.degreesToRadians( -120.00)));
                case Position_6  -> new Pose2d(new Translation2d(3.21, 3.86), new Rotation2d(Units.degreesToRadians(180.00)));
                case Position_7  -> new Pose2d(new Translation2d(3.21, 4.19), new Rotation2d(Units.degreesToRadians(180.00)));
                case Position_8  -> new Pose2d(new Translation2d(3.71, 5.05), new Rotation2d(Units.degreesToRadians(120.00)));
                case Position_9  -> new Pose2d(new Translation2d(3.99, 5.21), new Rotation2d(Units.degreesToRadians(120.00)));
                case Position_10 -> new Pose2d(new Translation2d(4.98, 5.21), new Rotation2d(Units.degreesToRadians(60.00)));
                case Position_11 -> new Pose2d(new Translation2d(5.27, 5.05), new Rotation2d(Units.degreesToRadians(60.00)));
                case Position_12 -> new Pose2d(new Translation2d(5.77, 4.19), new Rotation2d(Units.degreesToRadians(0.00)));
                default -> null;
            };
        }

        System.out.printf("Getting Selected Position: %s", selectedPosition);
        return Optional.of(selectedPosition);
    }

    public Optional<Pose2d> getRobotPositionForSelectedCoral() {
        return getRobotPositionForCoral(getCoralPosition());
    }
}
