// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.lib.math.Sequencer;
import frc.lib.util.Util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.FlywheelConstants.*;
import static frc.robot.Constants.AIM_DATA;

public class Flywheel extends SubsystemBase {
  private final WPI_TalonFX flywheelMasterController = new WPI_TalonFX(FLYWHEEL_MASTER_PORT);
  private final WPI_TalonFX flywheelSlaveController = new WPI_TalonFX(FLYWHEEL_SLAVE_PORT);
  
  //interpolates distances with datapoints
  private final CubicSplineInterpolate distanceToVelocity = new CubicSplineInterpolate();

  double shooterSetpoint = 0;

  //acceptable errors (to tell when the flywheel is up to speed)
  private final double velocityRange = 500,
                        accelerationRange = 100,
                        jerkRange = 100;

  //holds previous 3 flywheel velocities, for flywheel up to speed calculations
  double[] history = new double[3];

  /** Creates a new Shooter. */
  public Flywheel() {
    distanceToVelocity.setSamples(AIM_DATA[0], AIM_DATA[1]);

    flywheelSlaveController.follow(flywheelMasterController);

    flywheelMasterController.setInverted(true);
    flywheelSlaveController.setInverted(InvertType.OpposeMaster);

    configTalonGains(FLYWHEEL_kF, FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD);

    flywheelMasterController.setNeutralMode(NeutralMode.Coast);
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

  /**
   * use interpolation to set the flywheel speed for a target
   * @param distanceMeters the distance to the target
   */
  public void setVelcityForDistance(double distanceMeters){
    try{
      flywheelVelocity(distanceToVelocity.cubicSplineInterpolate(distanceMeters));
    } catch (ArrayIndexOutOfBoundsException e){
      flywheelPercent(0);
    }
  }

  /**
   * tell if the flywheel is up to speed
   * @return the flywheel speed
   */
  public boolean ready(){
    double[] decomposition = Sequencer.compute(history);
    return (
      Util.inRange(decomposition[0], velocityRange) &&
      Util.inRange(decomposition[1], accelerationRange) &&
      Util.inRange(decomposition[2], jerkRange)
      );
    // return (Math.abs(shooterSetpoint) - Math.abs(flywheelMasterController.getSelectedSensorVelocity())) < 500;
  }
  
  /**
   * set pidf gains for talons
   */
  public void configTalonGains(double kF, double kP, double kI, double kD){
    flywheelMasterController.config_kF(0, kF);
    flywheelMasterController.config_kP(0, kP);
    flywheelMasterController.config_kI(0, kI);
    flywheelMasterController.config_kD(0, kD);
    flywheelMasterController.configClosedloopRamp(2);
  }

  /**
   * @return flywheel speed in counts/100ms
   */
  public double getFlywheelVelocity(){
    return flywheelMasterController.getSelectedSensorVelocity();
  }

  /**
   * @return the difference between the target and current velocity
   */
  public double getFlywheelError(){
    return flywheelMasterController.getClosedLoopError();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("flywheel up to speed", ready());

    //shift shooter velocity into history array
    for(int i = 0; i < history.length - 1; i++){
      history[i] = history[i + 1];
    }
    history[history.length - 1] = Math.abs(shooterSetpoint) - Math.abs(flywheelMasterController.getSelectedSensorVelocity());
  }
}
