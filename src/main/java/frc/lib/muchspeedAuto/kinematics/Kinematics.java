package frc.lib.muchspeedAuto.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Point2d;
import frc.lib.math.PolarPoint2d;
import frc.robot.Constants;

public class Kinematics {
    private Pose2d pose;
    private final WheelSpeeds speeds;
    private double prevRotation = 0;
    private double avgDist = 0;

    private double[] shuffleboardData = {0,0};
    public Kinematics(Pose2d startPose, WheelSpeeds wheelSpeeds) {
        pose = startPose;
        speeds = wheelSpeeds;
    }
    public void update(double rotation) {
        speeds.update();
        Point2d changeVector = getDistVector(rotation);
        Point2d current = Point2d.fromPose(pose);
        current.transformBy(changeVector);

        shuffleboardData[0] = current.getX();
        shuffleboardData[1] = current.getY();

        pose = new Pose2d(current.getX(), current.getY(), new Rotation2d(rotation));
        avgDist += Math.abs(speeds.getAvgDelta());
    }

    public Rotation2d getDeltaRotaionFromEncoders(double left, double right){
        double rads = (right - left) / Constants.Drivetrain.DIST_BETWEEN_WHEELS;
        return new Rotation2d(rads);
    }

    public Pose2d getRobotPose(){
        return pose;
    }

    public double getAvgDistance(){
        return avgDist;
    }

    public void zero(){
        pose = new Pose2d(0,0,new Rotation2d(0));
    }

    private Point2d getDistVector(double rotation) {
        double dist = speeds.getAvgDelta();
        double averageRotation = (prevRotation + rotation) / 2;
        PolarPoint2d polarVector = new PolarPoint2d(dist, Rotation2d.fromDegrees(averageRotation));
        prevRotation = rotation;
        return PolarPoint2d.getCartesianPoint(polarVector);
    }

    public void putShuffleboard(){
        SmartDashboard.putNumber("Robot Pos X", shuffleboardData[0]);
        SmartDashboard.putNumber("Robot Pos Y", shuffleboardData[1]);
    }
}
