package frc.lib.muchspeedAuto.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.lib.math.LineMath;
import frc.lib.math.Point2d;
import frc.lib.math.Ratio;
import frc.lib.math.LineMath.Line;
import frc.robot.Constants;

public class InverseKinematics {
    public static Ratio get(Pose2d location, Point2d goTo){
        //get line between location and point to go to,
        //and line perpendicular to location and heading:
        Line pointsBisector = Line.fromEquation(LineMath.getBisectorOfPoints(Point2d.fromPose(location), goTo));
        pointsBisector.getEquation().print();
        
        location.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90)));
        Line linePerpHeading = Line.fromPose(location);
        linePerpHeading.getEquation().print();
        
        //get the point to rotate around:
        Point2d rotateAround = LineMath.getIntersectionOfLines(pointsBisector, linePerpHeading);
        // System.out.println(rotateAround.getX() + ", " + rotateAround.getY());

        //angle between (rot point - start) to (rot point - end):
        Line lineToDest = new Line();
        lineToDest.assign(rotateAround.getX(), rotateAround.getY(), goTo.getX(), goTo.getY());
        lineToDest.getEquation().print();

        Rotation2d angle = LineMath.angleBetween(linePerpHeading, lineToDest);
        // System.out.println(angle.toString());

        //calculate wheel locations
        double baseRad = LineMath.distanceBetween(Point2d.fromPose(location), rotateAround);
        // System.out.println(baseRad);

        double lRad = baseRad + (Constants.DIST_BETWEEN_WHEELS/2);
        double rRad = baseRad - (Constants.DIST_BETWEEN_WHEELS/2);

        //calculate wheel distances:
        double lCirc = lRad * Math.PI;
        double rCirc = rRad * Math.PI;

        double lDist = lCirc * (angle.getRadians() / Math.PI);
        double rDist = rCirc * (angle.getRadians() / Math.PI);
        // System.out.println(lDist);
        // System.out.println(rDist);

        Ratio wheelRates = new Ratio();
        wheelRates.calculate(lDist, rDist);
        System.out.println(wheelRates.toString());

        return wheelRates;
    }
    //testing
    public static void main(String args[]){
        Pose2d from = new Pose2d(1,0,new Rotation2d(-1));
        Point2d to = new Point2d(5,12);
        InverseKinematics.get(from,to);
    }
}
