// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HoodWIthLinearServo extends SubsystemBase {
    private final Servo hoodServo = new Servo(0); //todo move to constants i guess
    private static double move;

    //todo move to constants i guess
    private final double inchesFlywheelToServoBase = 5, //distance between flywheel and fixed mount of servo (for angle)
        inchesFlywheelToServoAttachment = 5, //distance between flywheel and servo attachment to hood (for angle)
        inchesMinLength = 0, //minimum extension inches (between mounting locations)
        inchesMaxLength = 0, //maximum extension
        //use these to adjust the range of servo travel
        percentMultiplier = 1,
        percentOffset = 0;

    private double setpoint = -1, 
        length = 0, 
        lastChangeTime,
        prevSetpoint = -1; //todo change starting length to collapsed length
    
    /** Creates a new Hood. */
    public HoodWIthLinearServo() {
        hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        lastChangeTime = Timer.getFPGATimestamp();
    }

    /**
     * set the hood servo position
     * @param position the position, with -1 being all the way in, and 1 being all the way out
     */
    public void setServoPercent(double position) {
        setpoint = position; //need to do setSpeed for position cause whatever
    }

    /**
     * set total length of the servo from the first mounting
     * hole to the second
     * @param inches inches to set to
     */
    public void setServoLengthInches(double inches) {
        setServoPercent(lengthToPercent(inches));
    }

    /**
     * set the actual angle of the hood
     * @param angle Rotation2d angle to set the hood to
     */
    public void setHoodAngle(Rotation2d angle) {
        double b = inchesFlywheelToServoAttachment;
        double c = inchesFlywheelToServoBase;
        double aSquared = (b * b) + (c * c) - (2 * b * c) * Math.cos(angle.getRadians());
        setServoLengthInches(Math.sqrt(aSquared));
    }

    public double getServoLength(){
        return length;
    }

    public double getServoPercent(){
        return setpoint;
    }

    /**
     * get the time since the position setpoint
     * of the servo was last changed
     * @return time since change in seconds
     */
    public double getTimeSinceChange(){
        return lastChangeTime;
    }

    //convert total servo extension in inches
    //to a percentage of extension
    private double lengthToPercent(double lengthInches) {
        double range = inchesMaxLength - inchesMinLength;
        double extensionLength = lengthInches - inchesMinLength;
        double halfPercent = extensionLength / range; //the percentage from 0 to 1;
        return (halfPercent * 2) - 1; //convert to -1 to 1
    }

    //convert percentage of servo extension
    //to the servo length in inches
    private double percentToLength(double percent) {
        double range = inchesMaxLength - inchesMinLength;
        double halfPercent = (percent + 1) / 2; // convert to 0 to 1
        double extensionLength = range * halfPercent;
        return extensionLength + inchesMinLength;
    }


    //Just for testing purposes, will get rewritten into something fancy later
    // public static double movedamount() {
    //     return move;
    // }
    // public void setmovedamount(double newgoto) {
    //     newgoto = move;
        
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //update servo positions
        hoodServo.setSpeed((setpoint * percentMultiplier) + percentOffset);
        length = percentToLength(setpoint);
        if(prevSetpoint != setpoint){
            lastChangeTime = Timer.getFPGATimestamp();
            prevSetpoint = setpoint;
        }
    
        // System.out.println("ServoMove:" + movedamount());

    }
}