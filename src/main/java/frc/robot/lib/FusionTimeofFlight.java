// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class FusionTimeofFlight  {
    private TimeOfFlight lidar;

    public FusionTimeofFlight(int id){
        lidar = new TimeOfFlight(id);
        lidar.setRangingMode(RangingMode.Short,20);
    }

    MedianFilter filter = new MedianFilter(10);
    public double Measure(){
        double distance = filter.calculate(lidar.getRange());
        SmartDashboard.putNumber("Lidar Distance", lidar.getRange());

        return distance;
    }
}