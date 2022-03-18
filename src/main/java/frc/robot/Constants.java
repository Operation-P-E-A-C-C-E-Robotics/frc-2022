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
    public static final class DriveTrainConstants{
        public static final int RIGHT_MASTER_PORT = 0,
                                RIGHT_SLAVE_PORT = 1,
                                LEFT_MASTER_PORT = 2,
                                LEFT_SLAVE_PORT = 3,
                                DRIVE_ENCODER_CPR = 2048;
        
        public static final double DRIVE_HIGH_GEAR_RATIO = 9.1,
                                    DRIVE_LOW_GEAR_RATIO = 24,
                                    WHEEL_DIAMETER_METERS = 0.1524; //TODO!!!
    }

    public static final class FlywheelConstants{
        public static final int FLYWHEEL_MASTER_PORT = 5,
                                FLYWHEEL_SLAVE_PORT = 4;  
        
        public static final double FLYWHEEL_kF = 0.05,
                                    FLYWHEEL_kP = 0.1,
                                    FLYWHEEL_kI = 0,
                                    FLYWHEEL_kD = 0;
    }
    
    public static final class TurretConstants{
        public static final int TURRET_CONTROLLER_PORT = 10;

        public static final double kFF = 0,
                                    kP = 0.1,
                                    kI = 0.00001,
                                    kD = 3,
                                   kIz = 0,
                                   MAX_OUTPUT = 1,
                                   MIN_OUTPUT = -1;

    }

    public static final class HoodConstants{
        //39 degrees max
        //TODO these are all wrong
        public static final double ENCODER_COUNTS_PER_CM = 2.5/100,
                                    FULLY_EXTENDED_COUNTS = 267,
                                    ATTACHMENT_POINT_RADIUS = 18,
                                    LOWEST_ANGLE = 18,
                                    kF = 0,
                                    kP = 100,
                                    kI = 0,
                                    kD = 0;
        public static final int     HOOD_CONTROLLER_PORT = 15;
    }
    
    public static final class BallHandlerConstants{
        public static final int INTAKE_CONTROLLER_PORT = 11,
                                TRAVERSAL_CONTROLLER_PORT = 7,
                                TRIGGER_CONTROLLER_PORT = 8;
        public static final double INTAKE_SPEED = 1,
                                    TRAVERSAL_SPEED = 1,
                                    TRIGGER_SPEED = 1;
    }

    public static final class ClimberConstants{
        public static final int CLIMBER_TOP_CONTROLLER_PORT = 16,
                                CLIMBER_BOTTOM_CONTROLLER_PORT = 17,
                                ARM_CONTORLLER_PORT = 18;
    }

    public static final double[][] AIM_DATA = {
        {0.2,   2.8,    3.5,    6}, //distances
        {6700,  7400,   8000,   10000}, //flywheel velocities
        {50,    170,    200,    226}, //hood angles
    };
}
