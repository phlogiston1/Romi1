package frc.lib.muchspeedAuto;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public interface AutoDrivetrain {
    public void voltageDrive(double lVolts, double rVolts);

    public void resetEncoders();

    public double getLeftDistance();

    public double getRightDistance();

    public DifferentialDriveWheelSpeeds getWheelSpeeds();
}
