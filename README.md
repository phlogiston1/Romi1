# Romi1 - 3461 romi code !In Progress!
## for challenge 1
### features:
- drivetrain with tank (left, right), arcade (forward, turn), position, and velocity drive options
- arcade drive with throttle max speed adjustment
- chezzy drive (cheesy drive, but misspelled), using joystick twist for quick turns
- reusable framework for creating ramsete paths ([/src/main/java/frc/lib/muchspeedAuto/paths/](/src/main/java/frc/lib/muchspeedAuto/paths/))
- reusable base for romi projects, with a drivetrain subsystem and simple drive commands, as well as default constants for ramsete path following

## to use libs:
- copy lib folder
- ### use default romi drivetrain:
    - subsystem has arcadeDrive(forward, turn), velocityDrive(left, right), and tankDrive(left,right) methods, in addition to getLeftPosition, getRightPosition, etc. Works with built-in drive commands
- ### custom drivetrain:
    - just implement 'AutoDrivetrain' and you can still use the auto libs!
- ### creating a path:
    - create a file extending PathBase
    - make a constructor, with parameters (RomiDrivetrain drivetrain, RobotPosition pos)
    - create a wpilib Trajectory [like in this tutorial](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html) - or import a pathweaver trajectory
    - run setTrajectory([your trajectory])
- ### run the path:
    - in RobotContainer create a new instance of the path, and return it in getAutonomousCommand
    - in Robot.java, change the autonomousInit command - change the line inside the if statement to read ```m_autonomosCommand.start();```

