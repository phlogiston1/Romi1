package frc.lib.muchspeedAuto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class RobotPosition {
    AutoDrivetrain dt;
    Gyro gr;
    DifferentialDriveKinematics kinematics;
    DifferentialDriveOdometry odometry;

    public RobotPosition(AutoDrivetrain drivetrain, Gyro gyro, double trackWidth) {
        dt = drivetrain;
        gr = gyro;
        kinematics = new DifferentialDriveKinematics(trackWidth);
        odometry = new DifferentialDriveOdometry(gyro.getHeading());
    }

    public void update() {
        odometry.update(gr.getHeading(), dt.getLeftDistance(), dt.getRightDistance());
    }

    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    public void zeroHeading() {
        gr.reset();
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return dt.getWheelSpeeds();
    }


    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, gr.getHeading());
    }



    public static interface Gyro{
        public Rotation2d getHeading();
        public void reset();
    }
}
