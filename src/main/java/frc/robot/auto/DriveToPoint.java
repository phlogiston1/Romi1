package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Point2d;
import frc.lib.math.PolarPoint2d;
import frc.robot.RobotState;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveToPoint extends CommandBase{
      @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;
  private final Point2d target;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToPoint(RomiDrivetrain subsystem, Point2d point) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    target = point;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pose2d location = RobotState.getRobotPose();
    // Point2d diff = new Point2d(target.getX() - location.getX(), target.getY() - location.getY());
    // PolarPoint2d angDist = Point2d.getPolarPoint(diff);
    m_subsystem.positionDrive(10, 10);
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
