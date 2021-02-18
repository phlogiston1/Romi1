package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleBoarder {
    public static class RobotState{
        public static double    DISTANCE_TRAVELED = 0,
                                ROBOT_POS_X = 0,
                                ROBOT_POS_Y = 0;
    }
    public static class Drivetrain{
        public static double    LEFT_TARGET = 0,
                                RIGHT_TARGET = 0,
                                LEFT_CURRENT = 0,
                                RIGHT_CURRENT = 0;
    }
    public static void initialize(){
        Shuffleboard.getTab("Robot State").add("Distance Traveled", RobotState.DISTANCE_TRAVELED);
        Shuffleboard.getTab("Robot State").add("pos x", RobotState.ROBOT_POS_X);
        Shuffleboard.getTab("Robot State").add("pos y", RobotState.ROBOT_POS_Y);
        Shuffleboard.getTab("Drivetrain").add("left target", Drivetrain.LEFT_TARGET);
        Shuffleboard.getTab("Drivetrain").add("right target", Drivetrain.RIGHT_TARGET);
        Shuffleboard.getTab("Drivetrain").add("left current", Drivetrain.LEFT_CURRENT);
        Shuffleboard.getTab("Drivetrain").add("right current", Drivetrain.RIGHT_CURRENT);
    }
}
