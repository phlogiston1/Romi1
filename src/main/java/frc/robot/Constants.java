// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static final double  DIST_BETWEEN_WHEELS = 5.5,
                                    DRIVETRAIN_KS       = 0,
                                    DRIVETRAIN_KV       = 0.03,
                                    DRIVETRAIN_KA       = 1,
                                    DRIVETRAIN_VEL_KP   = 0.03,
                                    DRIVETRAIN_VEL_KI   = 0.02,
                                    DRIVETRAIN_VEL_KD   = 0,
                                    DRIVETRAIN_POS_KP   = 0, 
                                    DRIVETRAIN_POS_KI   = 0, 
                                    DRIVETRAIN_POS_KD   = 0; 
    }
    public static final class Auto{
        public static final double  TRACK_W_METERS      = 0.142072613,
                                    KS                  = 0.929,
                                    KV                  = 6.33,
                                    KA                  = 0.0389,
                                    KP                  = 0.085,
                                    kI                  = 0,
                                    kD                  = 0,
                                    MAX_V               = 10,
                                    MAX_ACCEL           = 0.2,
                                    MAX_VEL             = 0.5,
                                    RAMSETE_B           = 2,
                                    RAMSETE_ZETA        = 0.7;
    }
}
