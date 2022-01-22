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
    public static final class DriveTrain{
        public static final int RIGHT_MASTER_PORT = 0,
                                RIGHT_SLAVE_PORT = 1,
                                LEFT_MASTER_PORT = 2,
                                LEFT_SLAVE_PORT = 3,
                                DRIVE_ENCODER_CPR = 0,
                                DRIVE_HIGH_GEAR_RATIO = 0;
    }

    public static final class Shooter{
        public static final int FLYWHEEL_CONTROLLER_PORT = 4,
                                TURRET_CONTROLLER_PORT = 5;  
        
        public static final double FLYWHEEL_kF = 0.05,
                                    FLYWHEEL_kP = 10,
                                    FLYWHEEL_kI = 0,
                                    FLYWHEEL_kD = 0.01;
    }
    
    public static final class Intake{
        public static final int INTAKE_CONTROLLER_PORT = 6;
        public static final double INTAKE_SPEED = 0.2;
    }

    public static final class Traversal{
        public static final int TRAVERSAL_CONTROLLER_PORT = 7;
    }
    
    //Limelight
    public static final double CAMERA_ANGLE = 0;

}
