package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.math.LineMath;
import frc.lib.math.Point2d;
import frc.lib.math.PolarPoint2d;
import frc.lib.math.Ratio;
import frc.lib.muchspeedAuto.kinematics.InverseKinematics;
import frc.lib.muchspeedAuto.path.Path;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.RomiDrivetrain;

public class TestAuto extends CommandBase{
      @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;
  // private final Point2d target;
  private Path path;
  private double div = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestAuto(RomiDrivetrain subsystem, Point2d point) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    // target = point;
    List<Point2d> points = new ArrayList<Point2d>();
    points.add(new Point2d(0,0));
    points.add(new Point2d(1,1));
    points.add(new Point2d(2,2));
    points.add(new Point2d(3,3));
    points.add(new Point2d(4,4));
    points.add(new Point2d(5,5));
    points.add(new Point2d(6,6));
    points.add(new Point2d(7,7));
    points.add(new Point2d(8,8));
    points.add(new Point2d(9,9));
    points.add(new Point2d(10,10));
    points.add(new Point2d(11,11));
    points.add(new Point2d(12,12));
    points.add(new Point2d(13,13));
    points.add(new Point2d(14,14));
    points.add(new Point2d(15,15));

    for(int i = 0; i < points.size() - 1; i++){
      div += LineMath.distanceBetween(points.get(i), points.get(i+1));
    }

    path = new Path(points);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Ratio kinematics = InverseKinematics.get(RobotState.getRobotPose(), target);
    // SmartDashboard.putString("kinematics ratio:", kinematics.toString());
    // /Ratio kinematics = InverseKinematics.get(RobotState.getRobotPose(), new Point2d(10,10));
    // double mult = LineMath.distanceBetween(Point2d.fromPose(RobotState.getRobotPose()), target);
    // SmartDashboard.putNumber("n1", kinematics.n1);
    // SmartDashboard.putNumber("n2", kinematics.n2);
    // m_subsystem.velocityDrive(kinematics.n1 * 3, -kinematics.n2 * 3);/ m_subsystem.velocityDrive(kinematics.n1*10,-kinematics.n2*10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pose2d location = RobotState.getRobotPose();
    // Point2d diff = new Point2d(target.getX() - location.getX(), target.getY() - location.getY());
    // PolarPoint2d angDist = Point2d.getPolarPoint(diff);
    SmartDashboard.putNumber("div", div);
    Point2d target = path.get((1 + RobotState.getRobotAvgDist()));

    SmartDashboard.putString("target", target.toString());

    Ratio kinematics = InverseKinematics.get(RobotState.getRobotPose(), new Point2d(10,10));
    double mult = LineMath.distanceBetween(Point2d.fromPose(RobotState.getRobotPose()), target);
    SmartDashboard.putNumber("n1", kinematics.n1);
    SmartDashboard.putNumber("n2", kinematics.n2);
    m_subsystem.velocityDrive(kinematics.n2 * 10 * RobotContainer.getInstance().driveJoy.getThrottle(), -kinematics.n1 * 5 * RobotContainer.getInstance().driveJoy.getThrottle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
