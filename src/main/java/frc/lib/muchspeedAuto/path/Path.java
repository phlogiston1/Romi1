package frc.lib.muchspeedAuto.path;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.lib.math.CubicSplineInterpolate;
import frc.lib.math.Point2d;

public class Path {
    private CubicSplineInterpolate x = new CubicSplineInterpolate(),y = new CubicSplineInterpolate();
    public Path(List<Point2d> points){
        double[] xAr = {}, yAr = {};
        for(int i = 0; i < points.size(); i++){
            xAr = append(xAr, points.get(i).getX());
            yAr = append(yAr, points.get(i).getY());
        }
        x.setSamples(makeInstances(xAr), xAr);
        y.setSamples(makeInstances(yAr), yAr);
    }
    public Point2d get(double distance){
        return new Point2d(x.cubicSplineInterpolate(distance), y.cubicSplineInterpolate(distance));
    }
    private double[] append(double[] ar, double el){
        double[] newar = Arrays.copyOf(ar, ar.length + 1);
        newar[newar.length - 1] = el;
        return newar;

    }
    private double[] makeInstances(double[] samples){
        double[] out = new double[samples.length];
        for(int i = 0; i < samples.length; i++){
            out[i] = i;
        }
        return out;
    }
    public static void main(String args[]){
        List<Point2d> testpoints = new ArrayList<Point2d>();
        testpoints.add(new Point2d(0,0));
        testpoints.add(new Point2d(1,0));
        testpoints.add(new Point2d(0,1));
        Path testPath = new Path(testpoints);
        for(double i = 0; i < 3; i += 0.01){
            System.out.println(testPath.x.cubicSplineInterpolate(i) + " " + testPath.y.cubicSplineInterpolate(i));
        }
    } 
}
