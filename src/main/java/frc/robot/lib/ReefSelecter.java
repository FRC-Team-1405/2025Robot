// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ReefSelecter {
    public enum CoralPosition {
        Level_1_Left,
        Level_2_Left,
        Level_3_Left,
        Level_4_Left,
        Level_1_Right,
        Level_2_Right,
        Level_3_Right,
        Level_4_Right,
    };

    public enum Direction {
        Left,
        Right,
    }

    public enum Level {
        Level_1,
        Level_2,
        Level_3,
        Level_4,
    }

    private Level level = Level.Level_4;
    private Direction direction = Direction.Left;
    private CoralPosition selected = updatePosition();

    public CoralPosition setLevel(Level level){
        this.level = level;
        return updatePosition();
    }

    public CoralPosition setDirection (Direction direction){
        this.direction = direction;
        return updatePosition();
    }

    public CoralPosition getPosition() {
            return selected;
    }

    public CoralPosition updatePosition() {
        switch (level) {
            case Level_1:
                selected = (direction == Direction.Left) ? (CoralPosition.Level_1_Left) : (CoralPosition.Level_1_Right);
                break;
            case Level_2:
                selected = (direction == Direction.Left) ? (CoralPosition.Level_2_Left) : (CoralPosition.Level_2_Right);
                break;
            case Level_3:
                selected = (direction == Direction.Left) ? (CoralPosition.Level_3_Left) : (CoralPosition.Level_3_Right);
                break;
            case Level_4:
                selected = (direction == Direction.Left) ? (CoralPosition.Level_4_Left) : (CoralPosition.Level_4_Right);
                break;
            default:
                selected =  (direction == Direction.Left) ? (CoralPosition.Level_1_Left) : (CoralPosition.Level_1_Right);
                break;            
        }

        // update shuffleboard
        SmartDashboard.putBoolean("Reef/Left L1", selected == CoralPosition.Level_1_Left);
        SmartDashboard.putBoolean("Reef/Right L1", selected == CoralPosition.Level_1_Right);
        SmartDashboard.putBoolean("Reef/Left L2", selected == CoralPosition.Level_2_Left);
        SmartDashboard.putBoolean("Reef/Right L2", selected == CoralPosition.Level_2_Right);
        SmartDashboard.putBoolean("Reef/Left L3", selected == CoralPosition.Level_3_Left);
        SmartDashboard.putBoolean("Reef/Right L3", selected == CoralPosition.Level_3_Right);
        SmartDashboard.putBoolean("Reef/Left L4", selected == CoralPosition.Level_4_Left);
        SmartDashboard.putBoolean("Reef/Right L4", selected == CoralPosition.Level_4_Right);
        return selected;

    }
}
