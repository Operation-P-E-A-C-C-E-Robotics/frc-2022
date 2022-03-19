// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.CubicSplineInterpolate;
import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.Constants.AIM_DATA;

public class Hood extends SubsystemBase {
    private final WPI_TalonSRX hoodMotor = new WPI_TalonSRX(HOOD_CONTROLLER_PORT);

    private CubicSplineInterpolate distanceToAngle = new CubicSplineInterpolate();

    double setpoint = 0;

    /** Creates a new Hood. */
    public Hood() {
        //to interpolate distances with data points
        distanceToAngle.setSamples(AIM_DATA[0], AIM_DATA[2]);

        hoodMotor.setInverted(true);

        configTalonGains(kF, kP, kI, kD);
    }

    /**
     * set hood motor percentage of power
     * @param percent percentage of full speed (-1 to 1)
     */
    public void setHoodSpeed(double percent){
        hoodMotor.set(percent);
    }

    /**
     * get whether the hood is in position to shoot
     * @return true if hood is ready to shoot
     */
    public boolean ready(){
        return (hoodMotor.getClosedLoopError()) < 3;
      }

    /**
     * set the hood position as a percentage of position
     * @param percent where to move to (0 to 1)
     */
    public void setHoodPosition(double percent){
        double counts = percent;
        setpoint = percent;
        setMotorPosition(counts);
    }

    /**
     * set hood extension in centimeters
     * @param cm (cm > 0)
     */
    public void setHoodExtension(double cm){
        double counts = cmToCounts(cm);
        setMotorPosition(counts);
    }

    /**
     * set the angle of the hood with 0 being streight up
     * @param angle
     */
    public void setHoodAngle(Rotation2d angle){
        double degreesFromZero = angle.getDegrees() - LOWEST_ANGLE;
        double rotations = degreesFromZero / 360;
        double cm = rotations * (2 * Math.PI * ATTACHMENT_POINT_RADIUS);
        setHoodExtension(cm);
    }

    /**
     * use interpolated data points to set the hood angle
     * @param distanceMeters distance to target
     */
    public void setHoodForDistance(double distanceMeters){
        try{
            setHoodPosition((distanceToAngle.cubicSplineInterpolate(distanceMeters)));
        } catch (ArrayIndexOutOfBoundsException e) {
            zero();
        }
    }

    /**
     * run the hood backwards till it zeroes
     */
    public void zero(){
        setHoodSpeed(-0.5); //limit switch should stop, zero when hits
    }

    /**
     * set pid gains for the hood controller
     */
    public void configTalonGains(double kF, double kP, double kI, double kD){
        hoodMotor.config_kF(0, kF);
        hoodMotor.config_kP(0, kP);
        hoodMotor.config_kI(0, kI);
        hoodMotor.config_kD(0, kD);
    }

    /**
     * set the motor position as encoder counts
     * @param counts the counts to set the motor to
     */
    public void setMotorPosition(double counts){
        counts = counts > FULLY_EXTENDED_COUNTS ? FULLY_EXTENDED_COUNTS : counts;
        counts = counts < 0 ? 0 : counts;
        if(counts == 0);// zero();
        else hoodMotor.set(ControlMode.Position, counts);
    }

    /**
     * zero the encoder
     */
    public void setEncoderZero(){
        hoodMotor.setSelectedSensorPosition(0);
    }

    /**
     * @return encoder counts from hood motor
     */
    public double getHoodPosition(){
        return hoodMotor.getSelectedSensorPosition();
    }

    private double cmToCounts(double cm){
        return cm / ENCODER_COUNTS_PER_CM;
    }

    @Override
    public void periodic() {
        //zero hood if limit switch closed
        if(hoodMotor.isRevLimitSwitchClosed() == 1) {
            hoodMotor.setSelectedSensorPosition(0);
        }
    }
}