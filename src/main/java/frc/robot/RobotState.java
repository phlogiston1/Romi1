package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.muchspeedAuto.RobotPosition;
import frc.lib.romiBase.sensors.RomiGyro;
import frc.lib.romiBase.subsystems.RomiDrivetrain;

public class RobotState {
    private static RomiGyro gyro = new RomiGyro();
    private static RomiDrivetrain dt = (RomiDrivetrain) RobotContainer.getInstance().getDrivetrain();
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
