// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.debloating.Ball;
import frc.lib.debloating.ColorSensor;
import frc.lib.debloating.Ball.Direction;

import static frc.robot.Constants.Intake.*;

public class BallHandler extends SubsystemBase {
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_CONTROLLER_PORT);
  private final WPI_TalonSRX triggerMotor = new WPI_TalonSRX(0); //todo do port number in constants
  //TODO this wont work for now because need to add multiple color sensors support
  private final ColorSensor traversalBallSensor = new ColorSensor(); 
  private final ColorSensor triggerBallSensor = new ColorSensor();
  private final Ball ball1 = new Ball();
  private final Ball ball2 = new Ball();
  
  boolean lastTraversalDirection = false; //todo false should be backwards, true forwards. make work like dat
  boolean wasBallAtTraversalPosition = false;

  /** Creates a new Intake. */
  public BallHandler() {
  }

  public void setPercent(double speed){
    System.out.println(speed);
    intakeMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake current", intakeMotor.getSupplyCurrent());
    
    if(intakeMotor.getMotorOutputPercent() > 0 || triggerMotor.getMotorOutputPercent() > 0) {
      ball1.setMotion(Direction.UPWARDS);
      ball2.setMotion(Direction.UPWARDS);
    } else if(intakeMotor.getMotorOutputPercent() < 0 || triggerMotor.getMotorOutputPercent() < 0) {
      ball1.setMotion(Direction.DOWNWARDS);
      ball2.setMotion(Direction.DOWNWARDS);
    }
    ball1.update(traversalBallSensor, triggerBallSensor, ball2);
    ball2.update(traversalBallSensor, triggerBallSensor, ball1);
  }
}
