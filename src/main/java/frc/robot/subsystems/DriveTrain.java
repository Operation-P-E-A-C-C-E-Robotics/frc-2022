// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveTrain.*;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX leftMasterController = new WPI_TalonFX(LEFT_MASTER_PORT);
  private final WPI_TalonFX leftSlaveController = new WPI_TalonFX(LEFT_SLAVE_PORT);
  private final WPI_TalonFX rightMasterController = new WPI_TalonFX(RIGHT_MASTER_PORT);
  private final WPI_TalonFX rightSlaveController = new WPI_TalonFX(RIGHT_SLAVE_PORT);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftSlaveController.follow(leftMasterController);
    rightSlaveController.follow(rightMasterController);

    //depending on gearboxes, motors could end up fighting. if so, change.
    leftMasterController.setInverted(true);
    rightMasterController.setInverted(false);
    leftSlaveController.setInverted(InvertType.FollowMaster);
    rightSlaveController.setInverted(InvertType.FollowMaster);
  }

  /**
   * Tank Drive - Send raw values to the DriveTrain, no differential mechanics involved, Useful for autonomous
   * @param left - Power to send to the left side of the drive train, -1 - 1
   * @param right - Power to send to the riht side of the drive train, -1 - 1
   */
  public void percentDrive(double left, double right) {
    leftMasterController.set(left);
    rightMasterController.set(right);
 }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Speed:", leftMasterController.get());
    SmartDashboard.putNumber("Right Speed:", rightMasterController.get());
    //Puts the speed being sent to the motors on the dashboard, helpful for diagnostics
  }
}
