// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.math.Point2d;
import frc.robot.auto.DriveToPoint;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private HashMap<String, Subsystem> subsystems = new HashMap<String,Subsystem>();
  private HashMap<String, Command> commands = new HashMap<String, Command>();
  //private RobotState robotState = new RobotState();
  Joystick driveJoy = new Joystick(0);

  private static RobotContainer instance;

  private final ArcadeDrive driveCommand = new ArcadeDrive(m_romiDrivetrain, driveJoy);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    subsystems.put("drivetrain", m_romiDrivetrain);
    commands.put("drive", driveCommand);

    if(instance == null) instance = this;
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(driveCommand);
  }

  public Subsystem getSubsystem(String name){
    return subsystems.get(name);
  }

  public RomiDrivetrain getDrivetrain(){
    return m_romiDrivetrain;
  }

  public Command getCommand(String name){
    return commands.get(name);
  }

  public static RobotContainer getInstance(){
    if(instance == null) instance = new RobotContainer();
    return instance;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveToPoint(m_romiDrivetrain, new Point2d(0,0));
  }

  public Command getDriveCommand(){
    return driveCommand;
  }
}
