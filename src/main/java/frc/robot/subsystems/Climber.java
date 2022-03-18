
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private WPI_TalonFX liftMasterController = new WPI_TalonFX(CLIMBER_TOP_CONTROLLER_PORT); 
  private WPI_TalonFX liftSlaveController = new WPI_TalonFX(CLIMBER_BOTTOM_CONTROLLER_PORT);

  private WPI_TalonFX armWinch = new WPI_TalonFX(ARM_CONTORLLER_PORT);
  private final double liftRaisedCounts = 0;

  /** Creates a new Climber. */
  public Climber() {
    liftSlaveController.follow(liftMasterController);

    liftMasterController.setInverted(false);
    liftSlaveController.setInverted(InvertType.FollowMaster);
  
    liftMasterController.config_kP(0,0);
    liftMasterController.config_kI(0,0);
    liftMasterController.config_kD(0,0);
    liftMasterController.config_kF(0,0);
    liftMasterController.configMotionAcceleration(0);
    liftMasterController.configMotionCruiseVelocity(0);
    liftMasterController.configMotionSCurveStrength(0);

  }
 
  public void setLiftPercent(double percent){
    liftMasterController.set(percent);
  }

  public void liftUp(){
    liftMasterController.set(ControlMode.MotionMagic, liftRaisedCounts);
  }

  public void liftDown(){
    liftMasterController.set(ControlMode.MotionMagic, 0);
  }

  public void setArmPercent(double percent){
    armWinch.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
