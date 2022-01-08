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
  public static WPI_TalonSRX m_leftFollow = new WPI_TalonSRX(Constants.Lfollow);
  public static WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(Constants.Rdrive);
  public static WPI_TalonSRX m_rightFollow = new WPI_TalonSRX(Constants.Rfollow);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Speed:", m_leftMotor.get());
    SmartDashboard.putNumber("Right Speed:", m_rightMotor.get());
    //Puts the speed being sent to the motors on the dashboard, helpful for diagnostics
  }




  /**
   * Tank Drive - Send raw values to the DriveTrain, no differential mechanics involved, Useful for autonomous
   * @param left - Power to send to the left side of the drive train, -1 - 1
   * @param right - Power to send to the riht side of the drive train, -1 - 1
   */
  public void tankdrive(double left, double right) {
  m_leftMotor.set(left);
  m_rightMotor.set(right);
  m_leftFollow.set(left);
  m_rightFollow.set(right);

 }

 /**
  * Arcade Drive - Accepts joystick inputs, Takes the joystick's forward and backward, left and right and converts it into a percentage and passes it to the motors giving beter control to the driver
  * @param stickfb
  * @param sticklr
  */
 public void arcadedrive(double stickfb, double sticklr) {
  
  m_rightMotor.set(sticklr - stickfb);
  m_rightFollow.set(sticklr - stickfb);
  m_leftMotor.set(stickfb + sticklr);
  m_leftFollow.set(stickfb + sticklr);

 }


/**
 * Sets all motors to 0% output
 */
public void allstop() {
  m_rightMotor.set(0);
  m_leftMotor.set(0);
  m_leftFollow.set(0);
  m_rightFollow.set(0);
 }

}
