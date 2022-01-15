// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX flywheelMasterController = new WPI_TalonFX(FLYWHEEL_CONTROLLER_PORT);
  private final WPI_TalonFX flywheelSlaveController = new WPI_TalonFX(FLYWHEEL_CONTROLLER_PORT);
  private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(TURRET_CONTROLLER_PORT);

  /** Creates a new Shooter. */
  public Shooter() {
    flywheelSlaveController.follow(flywheelMasterController);

    //CHANGE IF FIGHTING
    flywheelMasterController.setInverted(false);
    flywheelSlaveController.setInverted(InvertType.FollowMaster);

    configTalonGains(FLYWHEEL_kF, FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD);
    SmartDashboard.putNumber("flywheel kf", FLYWHEEL_kF);
    SmartDashboard.putNumber("flywheel kp", FLYWHEEL_kP);
    SmartDashboard.putNumber("flywheel ki", FLYWHEEL_kI);
    SmartDashboard.putNumber("flywheel kd", FLYWHEEL_kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Flywheel Speed", shootMotor.get());
    SmartDashboard.putNumber("Flywheel Aim Speed:", turretMotor.get());
    configTalonGains(SmartDashboard.getNumber("flywheel kf", FLYWHEEL_kF), 
                    SmartDashboard.getNumber("flywheel kp", FLYWHEEL_kP), 
                    SmartDashboard.getNumber("flywheel ki", FLYWHEEL_kI),
                    SmartDashboard.getNumber("flywheel kd", FLYWHEEL_kD));
  }

  public void flywheelPercent(double speed) {
    flywheelMasterController.set(speed);
  }

  public void turretPercent(Double speed) {
    turretMotor.set(speed);
  }

  public void flywheelVelocity(double velocity){
    flywheelMasterController.set(ControlMode.Velocity, velocity);
  }

  private void configTalonGains(double kF, double kP, double kI, double kD){
    flywheelMasterController.config_kF(0, kF);
    flywheelMasterController.config_kP(0, kP);
    flywheelMasterController.config_kI(0, kI);
    flywheelMasterController.config_kD(0, kD);
    flywheelMasterController.configClosedloopRamp(10);
  }

}