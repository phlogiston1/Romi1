package frc.lib.muchspeedAuto.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.lib.math.LineMath;
import frc.lib.math.Point2d;
import frc.lib.math.LineMath.Line;
import frc.lib.math.LineMath.LineEquation;

public class InverseKinematics {
    public void get(Pose2d location, Point2d goTo){
        //get line between location and point to go to,
        //and line perpendicular to location and heading:
        Line pointsBisector = Line.fromEquation(LineMath.getBisectorOfPoints(Point2d.fromPose(location), goTo));
        pointsBisector.print();
        location.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90)));
        Line linePerpHeading = Line.fromPose(location);
        linePerpHeading.getEquation().print();
        Point2d rotateAround = LineMath.getIntersectionOfLines(pointsBisector, linePerpHeading);
        System.out.println(rotateAround.getX() + " " + rotateAround.getY());
    } 
    //testing
    public static void main(String args[]){
        Pose2d from = new Pose2d(0,0,new Rotation2d(0));
        Point2d to = new Point2d(10,5);
        new InverseKinematics().get(from,to);
    }
}
