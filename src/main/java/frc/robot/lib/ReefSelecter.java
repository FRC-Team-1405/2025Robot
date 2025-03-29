// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.lang.System.Logger.Level;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elavator;

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

    private Elavator.ElevationLevel level = Elavator.ElevationLevel.Level_4;
    private CoralLevel coralLevel = CoralLevel.Level_4;
    private Coral coralSelected = Coral.Position_1;

    public ReefSelecter(){
        updatePosition();
    }
    public void setLevel(Elavator.ElevationLevel level){
        this.level = level;
        updatePosition();
    }

    public Elavator.ElevationLevel getLevel() {
        return level;
    }

    public void levelUp() {
        level = switch(level) {
            case Level_1 -> Elavator.ElevationLevel.Level_2;
            case Level_2 -> Elavator.ElevationLevel.Level_3;
            case Level_3 -> Elavator.ElevationLevel.Level_4;
            default -> level;
        };
        updatePosition();
    }
    
    public void levelDown(){
        level = switch(level){
            case Level_4 -> Elavator.ElevationLevel.Level_3;
            case Level_3 -> Elavator.ElevationLevel.Level_2;
            case Level_2 -> Elavator.ElevationLevel.Level_1;
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
    }
}
