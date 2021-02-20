package frc.lib.muchspeedAuto.kinematics;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.romiBase.subsystems.RomiDrivetrain;

public class WheelSpeeds {
    private double prevLDist = 0,
                    prevRDist = 0,
                    deltaL,
                    deltaR;
    RomiDrivetrain supplier;
    double[] shfDt = {0,0,0};
    public WheelSpeeds(RomiDrivetrain drive){
        supplier = drive;
        prevLDist = supplier.getLeftDistanceInch();
        prevRDist = supplier.getRightDistanceInch();
    }
    public void update() {
        double L = supplier.getLeftDistanceInch();
        double R = supplier.getRightDistanceInch();
        deltaL = L - prevLDist;
        deltaR = R - prevRDist;
        prevLDist = L; 
        prevRDist = R;

        shfDt[0] = deltaL;
        shfDt[1] = deltaR;
        shfDt[2] = getAvgDelta();
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
    public void putShuffleboard(){
        SmartDashboard.putNumber("A left rotation", shfDt[0]);
        SmartDashboard.putNumber("A right rotation", shfDt[1]);
        SmartDashboard.putNumber("avg A", shfDt[2]);
    }
}
