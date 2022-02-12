// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.CubicSplineInterpolate;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX flywheelMasterController = new WPI_TalonFX(FLYWHEEL_MASTER_PORT);
  private final WPI_TalonFX flywheelSlaveController = new WPI_TalonFX(FLYWHEEL_SLAVE_PORT);

  private CubicSplineInterpolate distanceToVelocity = new CubicSplineInterpolate();

  double shooterSetpoint = 0;

  /** Creates a new Shooter. */
  public Shooter() {
    double[] distances = {2,3};
    double[] velocities = {1000,2000};

    distanceToVelocity.setSamples(distances, velocities);

    flywheelSlaveController.follow(flywheelMasterController);

    //CHANGE IF FIGHTING
    flywheelMasterController.setInverted(false);
    flywheelSlaveController.setInverted(InvertType.OpposeMaster);

    configTalonGains(FLYWHEEL_kF, FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD);

    flywheelMasterController.setNeutralMode(NeutralMode.Coast);
    flywheelMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 35, 10));


    SmartDashboard.putNumber("shooter kp", FLYWHEEL_kP);
    SmartDashboard.putNumber("shooter ki", FLYWHEEL_kI);
    SmartDashboard.putNumber("shooter kd", FLYWHEEL_kD);
    SmartDashboard.putNumber("shooter kf", FLYWHEEL_kF);
  }
  
  /**
   * set the flywheel power
   * @param speed the speed (-1 to 1)
   */
  public void flywheelPercent(double speed) {
    flywheelMasterController.set(speed);
  }
  /**
   * set the flywheel velocity with the built in pidf control
   * @param velocity the velocity to set to in sensor units/100ms
   */
  public void flywheelVelocity(double velocity){
    flywheelMasterController.set(ControlMode.Velocity, velocity);
    shooterSetpoint = velocity;
  }

  public void setVelcityForDistance(double distanceMeters){
    try{
      flywheelVelocity(distanceToVelocity.cubicSplineInterpolate(distanceMeters));
    } catch (ArrayIndexOutOfBoundsException e){
      flywheelPercent(0);
    }
  }

  public boolean ready(){
    return (shooterSetpoint - flywheelMasterController.getSelectedSensorVelocity()) < 10;
  }
  
  /**
   * set pidf gains for talons
   */
  private void configTalonGains(double kF, double kP, double kI, double kD){
    flywheelMasterController.config_kF(0, kF);
    flywheelMasterController.config_kP(0, kP);
    flywheelMasterController.config_kI(0, kI);
    flywheelMasterController.config_kD(0, kD);
    flywheelMasterController.configClosedloopRamp(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter velocity", flywheelMasterController.getSelectedSensorVelocity());
    SmartDashboard.putNumber("shooter motor 1 current", flywheelMasterController.getSupplyCurrent());
    SmartDashboard.putNumber("shooter motor 2 current", flywheelSlaveController.getSupplyCurrent());
    SmartDashboard.putNumber("shooter velocity error", flywheelMasterController.getClosedLoopError());

    SmartDashboard.putBoolean("shooter ready", ready());

    configTalonGains(SmartDashboard.getNumber("shooter kf", FLYWHEEL_kF),
              SmartDashboard.getNumber("shooter kp", FLYWHEEL_kP),
              SmartDashboard.getNumber("shooter ki", FLYWHEEL_kI),
              SmartDashboard.getNumber("shooter kd", FLYWHEEL_kD));
  }
}
