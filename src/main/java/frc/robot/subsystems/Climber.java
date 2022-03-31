
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private WPI_TalonFX liftMasterController = new WPI_TalonFX(CLIMBER_TOP_CONTROLLER_PORT); 
  private WPI_TalonFX liftSlaveController = new WPI_TalonFX(CLIMBER_BOTTOM_CONTROLLER_PORT);

  // private WPI_TalonFX armWinch = new WPI_TalonFX(ARM_CONTORLLER_PORT);
  private DoubleSolenoid armSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 4, 5); //TODO numbers
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

    liftMasterController.setNeutralMode(NeutralMode.Brake);

  }
 
  public void setLiftPercent(double percent){
    liftMasterController.set(percent);
  }

  public void armsOut(){
    armSolenoid.set(Value.kForward);
  }

  public void armsIn(){
    armSolenoid.set(Value.kReverse);
  }

  public void armsToggle(){
    armSolenoid.toggle();
  }

  public double getLiftPosition(){
    return liftMasterController.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(liftMasterController.isFwdLimitSwitchClosed() == 1){
      liftMasterController.setSelectedSensorPosition(0);
    }
  }
}
