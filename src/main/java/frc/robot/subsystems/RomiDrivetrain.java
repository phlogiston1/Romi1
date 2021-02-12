// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    SmartDashboard.putNumber("kp", 0.3);
    SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kd", 0);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  public void arcadeDrive(double speed, double turn) {
    tankDrive(speed - turn, speed + turn);
  }

  double kP = 0, kI = 0, kD = 0;
  double li = 0, ri = 0;
  double prevREr = 0, prevLEr = 0;
  public void positionDrive(double leftPosition, double rightPosition){
    kP = SmartDashboard.getNumber("kp", 0);
    kI = SmartDashboard.getNumber("ki", 0);
    kD = SmartDashboard.getNumber("kd", 0);
    double lError = getLeftDistanceInch() - leftPosition;
    double rError = getRightDistanceInch() - rightPosition;

    double lp = kP * lError;
    double rp = kP * rError;

    li += kI;
    ri += kI;

    double ld = (lError - prevLEr) * kD;
    double rd = (prevREr - rError) * kD;

    tankDrive(-(lp + li + ld), rp + ri + rd);

    prevLEr = lError;
    prevREr = rError;
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    RobotState.update();
  
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
