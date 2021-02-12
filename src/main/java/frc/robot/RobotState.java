package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.muchspeedAuto.kinematics.Kinematics;
import frc.lib.muchspeedAuto.kinematics.WheelSpeeds;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

public class RobotState {
    private static RomiGyro gyro = new RomiGyro();
    private static RomiDrivetrain dt = (RomiDrivetrain) RobotContainer.getInstance().getSubsystem("drivetrain");
    private static WheelSpeeds speds = new WheelSpeeds(dt);
    private static Kinematics kin = new Kinematics(new Pose2d(0,0,new Rotation2d(gyro.getAngleZ())), speds);
    public static void update(){
        kin.update(gyro.getAngleZ());
    }
    public static void zeroKinematics(){
        kin.zero();
    }
    public static Pose2d getRobotPose(){
        return kin.getRobotPose();
    }
}