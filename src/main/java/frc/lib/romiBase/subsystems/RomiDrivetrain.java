// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.romiBase.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.muchspeedAuto.AutoDrivetrain;
import frc.robot.RobotState;
import static frc.robot.Constants.Drivetrain.*;

public class RomiDrivetrain extends SubsystemBase implements AutoDrivetrain {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  // private static double lDriftAccumulator = 0;
  // private static double rDriftAccumulator = 0;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);
  public DifferentialDrive dDrive = new DifferentialDrive(m_leftMotor, m_rightMotor); 
  private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(DRIVETRAIN_KS,DRIVETRAIN_KV,DRIVETRAIN_KA);
  private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(DRIVETRAIN_KS,DRIVETRAIN_KV,DRIVETRAIN_KA);
  PIDController lPidController = new PIDController(DRIVETRAIN_VEL_KP, DRIVETRAIN_VEL_KI, DRIVETRAIN_VEL_KD);
  PIDController rPidController = new PIDController(DRIVETRAIN_VEL_KP, DRIVETRAIN_VEL_KI, DRIVETRAIN_VEL_KD);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_leftMotor.enableDeadbandElimination(true);
    m_rightMotor.enableDeadbandElimination(true);
    resetEncoders();
  }
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
    dDrive.feed();
  }

  public void arcadeDrive(double speed, double turn) {
    velocityDrive((speed - turn)*20,(speed + turn)*20);
  }

  public void velocityDrive(double lSpeed, double rSpeed){

    double lpid = lPidController.calculate(-getLeftVelocity(),lSpeed);
    double lffd = leftFeedforward.calculate(-getLeftVelocity());
    double rpid = rPidController.calculate(getRightVelocity(),rSpeed);
    double rffd = rightFeedforward.calculate(getRightVelocity());

    SmartDashboard.putNumber("lspeed", lSpeed);
    SmartDashboard.putNumber("rspeed", rSpeed);
    SmartDashboard.putNumber("lvel", getLeftVelocity());
    SmartDashboard.putNumber("rvel", getRightVelocity());

    tankDrive(-(lpid + lffd), -(rpid + rffd));
  }

  double kP = DRIVETRAIN_POS_KP, kI = DRIVETRAIN_POS_KI, kD = DRIVETRAIN_POS_KD;
  double li = 0, ri = 0;
  double prevREr = 0, prevLEr = 0;
  public void positionDrive(double leftPosition, double rightPosition){
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

  public void voltageDrive(double lVolts, double rVolts){
    m_leftMotor.setVoltage(lVolts);
    m_rightMotor.setVoltage(-rVolts);
    dDrive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getLeftDistance(){
    return Units.inchesToMeters(getLeftDistanceInch());
  }

  public double getRightDistance(){
    return Units.inchesToMeters(getRightDistanceInch());
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocity(){
    return m_leftEncoder.getRate();
  }
  public double getRightVelocity(){
    return m_rightEncoder.getRate();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(getLeftDistanceInch()), Units.inchesToMeters(getRightDistanceInch()));
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
