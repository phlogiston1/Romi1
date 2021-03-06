package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

public class RobotState {
    private static RomiGyro gyro = new RomiGyro();
    private static RomiDrivetrain dt = (RomiDrivetrain) RobotContainer.getInstance().getSubsystem("drivetrain");
    private static Joystick drvJoy = RobotContainer.getInstance().driveJoy;
    //private static WheelSpeeds speds = new WheelSpeeds(dt);
    public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Auto.TRACK_W_METERS);
    public static DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getHeading());
    //private static Kinematics kin = new Kinematics(new Pose2d(0,0,new Rotation2d(gyro.getAngleZ())), speds);
    //private static NetworkTableEntry distance = Shuffleboard.getTab("Robot State").add("distance traveled", kin.getAvgDistance()).getEntry();

    public static void init(){
        SmartDashboard.putNumber("xoffset", 0);
        SmartDashboard.putNumber("yoffset", 0);
    }

    public static void update(){
        //kin.update(gyro.getAngleZ());
        //distance.setNumber(kin.getAvgDistance());
        odometry.update(getHeading(), dt.getLeftDistance(),dt.getRightDistance());
        SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());

        if(drvJoy.getRawButton(2)){
            //setOdometryStart();
        }
    }

    public static void setOdometryStart(Pose2d startPose){
            dt.resetEncoders();
            odometry.resetPosition(startPose, gyro.getHeading());

    }

    public static Pose2d getCurrentPose(){
        return odometry.getPoseMeters();
    }
    public static void zeroHeading(){
        gyro.reset();
    }
    public static void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, getHeading());
    }

    // public static double getRobotAvgDist(){
    //     //return kin.getAvgDistance();
    // }

    // public static Pose2d getRobotPose(){
    //     //return kin.getRobotPose();
    // }

    public static Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-gyro.getAngleZ());
    }
}
