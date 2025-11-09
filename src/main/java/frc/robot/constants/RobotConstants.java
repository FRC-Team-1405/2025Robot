package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class RobotConstants {

    // using 36in instead of actual 35in, because i was seeing small issues with the position calculation
    // that would cause our target position to be inside of the reef by half an inch.
    // the pose2d objects round to ~0.5in and that caused a slight misalignment.
    // double ROBOT_WIDTH = Units.inchesToMeters(36) / 2;

    public static final double ROBOT_WIDTH = Units.inchesToMeters(38); // todo remove when you fix autopilot
    public static final double HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2.0;
}
