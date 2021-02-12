package frc.lib.muchspeedAuto.kinematics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RomiDrivetrain;

public class WheelSpeeds {
    private double prevLDist = 0,
                    prevRDist = 0,
                    deltaL,
                    deltaR;
    RomiDrivetrain supplier;
    public WheelSpeeds(RomiDrivetrain drive){
        supplier = drive;
        prevLDist = supplier.getLeftDistanceInch();
        prevRDist = supplier.getRightDistanceInch();
    }
    public void update() {
        SmartDashboard.putNumber("prevL", prevLDist);
        double L = supplier.getLeftDistanceInch();
        double R = supplier.getRightDistanceInch();
        deltaL = L - prevLDist;
        deltaR = R - prevRDist;
        prevLDist = L; 
        prevRDist = R;
        SmartDashboard.putNumber("L", L);

        SmartDashboard.putNumber("deltal: ", deltaL);
        SmartDashboard.putNumber("deltaR: ", deltaR);
        SmartDashboard.putNumber("avg: ", getAvgDelta());
    }
    public double getDeltaL() {
        return deltaL;
    }
    public double getDeltaR() {
        return deltaR;
    }
    public double getAvgDelta(){
        return (deltaL + deltaR) / 2;
    }
}
