// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.CubicSplineInterpolate;
import static frc.robot.Constants.Hood.*;

public class Hood extends SubsystemBase {
    private final WPI_TalonSRX hoodMotor = new WPI_TalonSRX(0); //todo port number

    private CubicSplineInterpolate distanceToAngle = new CubicSplineInterpolate();

    /** Creates a new Hood. */
    public Hood() {
        hoodMotor.setInverted(false); //change so positive = forward
        configTalonGains(0, 0, 0, 0); //todo change duh
    }

    /**
     * set hood motor percentage of power
     * @param percent percentage of full speed (-1 to 1)
     */
    public void setHoodSpeed(double percent){
        hoodMotor.set(percent);
    }

    /**
     * set the hood position as a percentage of position
     * @param percent where to move to (0 to 1)
     */
    public void setHoodPosition(double percent){
        double counts = percent * FULLY_EXTENDED_COUNTS;
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

    public void zero(){
        setHoodSpeed(-0.1); //limit switch should stop, zero when hits
    }

    public void configTalonGains(double kF, double kP, double kI, double kD){
        hoodMotor.config_kF(0, kF);
        hoodMotor.config_kP(0, kP);
        hoodMotor.config_kI(0, kI);
        hoodMotor.config_kD(0, kD);
    }

    private void setMotorPosition(double counts){
        counts = counts > FULLY_EXTENDED_COUNTS ? FULLY_EXTENDED_COUNTS : counts;
        counts = counts < 0 ? 0 : counts;
        hoodMotor.set(ControlMode.Position, counts);
    }

    private double countsToCM(double counts){
        return counts / ENCODER_COUNTS_PER_CM;
    }

    private double cmToCounts(double cm){
        return cm * ENCODER_COUNTS_PER_CM;
    }

    @Override
    public void periodic() {
        if(hoodMotor.isRevLimitSwitchClosed() == 1) hoodMotor.setSelectedSensorPosition(0);
    }
}