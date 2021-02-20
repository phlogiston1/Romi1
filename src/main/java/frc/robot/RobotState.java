package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.muchspeedAuto.RobotPosition;
import frc.lib.romiBase.sensors.RomiGyro;
import frc.lib.romiBase.subsystems.RomiDrivetrain;

public class RobotState {
    private static RomiGyro gyro = new RomiGyro();
    private static RomiDrivetrain dt = (RomiDrivetrain) RobotContainer.getInstance().getSubsystem("drivetrain");
    private static Joystick drvJoy = RobotContainer.getInstance().driveJoy;
    private static RobotPosition rPos = new RobotPosition(dt, gyro, Constants.Auto.TRACK_W_METERS);

    public static void update(){
        rPos.update();
        SmartDashboard.putNumber("X", rPos.getCurrentPose().getX());
        SmartDashboard.putNumber("Y", rPos.getCurrentPose().getY());

        if(drvJoy.getRawButton(2)){
            gyro.reset();
            dt.resetEncoders();
        }
    }

    public static RobotPosition getPosition(){
        return rPos;
    }
}
