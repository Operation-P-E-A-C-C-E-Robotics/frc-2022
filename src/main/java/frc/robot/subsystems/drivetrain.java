// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  public drivetrain() {}
  public static WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(Constants.Ldrive);
  public static WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(Constants.Rdrive);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Speed:", m_leftMotor.get());
    SmartDashboard.putNumber("Right Speed:", m_rightMotor.get());
    //Puts the speed being sent to the motors on the dashboard, helpful for diagnostics
  }


//Tank Drive, takes values from -1 to 1, sets the side you pass to the speed passed
//ie tankdrive(1,-1); would make the robot spin in a circle
  public void tankdrive(double left, double right) {
  m_leftMotor.set(left);
  m_rightMotor.set(right);
 }

//Differential Drive, for taking joystick inputs
//Takes the joystick's forward and backward, left and right and converts it into a percentage and passes it to the motors giving beter control to the driver
 public void arcadedrive(double stickfb, double sticklr) {
  
  m_rightMotor.set(sticklr - stickfb);
  m_leftMotor.set(stickfb + sticklr);
 }

//Sets all motors to 0% output
public void allstop() {
  m_rightMotor.set(0);
  m_leftMotor.set(0);
 }

}
