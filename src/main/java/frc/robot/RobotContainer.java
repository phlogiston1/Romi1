// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.math.Point2d;
import frc.robot.commands.ChezzyDrive;
import frc.robot.commands.auto.paths.PathBase;
import frc.robot.commands.auto.paths.TestPath;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private HashMap<String, Subsystem> subsystems = new HashMap<String, Subsystem>();
  private HashMap<String, Command> commands = new HashMap<String, Command>();
  // private RobotState robotState = new RobotState();
  public Joystick driveJoy = new Joystick(0);

  private static RobotContainer instance;

  private final ChezzyDrive driveCommand = new ChezzyDrive(m_romiDrivetrain, driveJoy);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    subsystems.put("drivetrain", m_romiDrivetrain);
    commands.put("drive", driveCommand);

    if (instance == null)
      instance = this;
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(driveCommand);
  }

  public Subsystem getSubsystem(String name) {
    return subsystems.get(name);
  }

  public RomiDrivetrain getDrivetrain() {
    return m_romiDrivetrain;
  }

  public Command getCommand(String name) {
    return commands.get(name);
  }

  public static RobotContainer getInstance() {
    if (instance == null)
      instance = new RobotContainer();
    return instance;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public PathBase getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    try {
      return new TestPath(m_romiDrivetrain);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return null;
  }

  public Command getDriveCommand(){
    return driveCommand;
  }
}
