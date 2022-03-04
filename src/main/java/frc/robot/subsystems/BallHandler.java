// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.debloating.Ball;
import frc.lib.debloating.ColorSensor;
import frc.lib.debloating.Ball.Direction;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Traversal.*;

public class BallHandler extends SubsystemBase {
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_CONTROLLER_PORT);
  private final WPI_TalonSRX traversalMotor = new WPI_TalonSRX(TRAVERSAL_CONTROLLER_PORT);
  private final WPI_TalonSRX triggerMotor = new WPI_TalonSRX(TRIGGER_CONTROLLER_PORT); //todo do port number in constants
  //TODO this wont work for now because need to add multiple color sensors support
  private final ColorSensor traversalBallSensor = new ColorSensor(); 
  private final ColorSensor triggerBallSensor = new ColorSensor();
  private boolean canRunIntake = false,
                  armsDown = false;
  private double armsLoweredTimer = 0;


  //Arm Pnuematics
  private final DoubleSolenoid arms = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 2, 3);

  /** Creates a new Intake. */
  public BallHandler() {
    traversalMotor.setInverted(true);
    setMotorConstants(intakeMotor, 0, 0, 0);
    setMotorConstants(traversalMotor, 0, 0, 0);
    setMotorConstants(triggerMotor, 0, 0, 0);
    traversalMotor.configContinuousCurrentLimit(5);
    armsUp();
  }

  private void setMotorConstants(WPI_TalonSRX m, double kp, double ki, double kd){
    m.config_kP(0, kp);
    m.config_kI(0, ki);
    m.config_kD(0, kd);
  }

  public void setIntake(double speed){
    //System.out.println(speed);
    if (arms.get() == Value.kForward ) {intakeMotor.set(speed);}
    else {intakeMotor.set(0);}
    }

  public void setTraversal(double speed){
    traversalMotor.set(speed);
  }

  public void setTrigger(double speed){
    triggerMotor.set(speed);
  }

  public void setAll(double speed){
    setIntake(speed * 1);
    setTraversal(speed * 0.5);
    setTrigger(speed * 0.2);
  }
  
  public void setIntakePosition(double position){
    intakeMotor.set(ControlMode.Position, position);
  }

  public void setTraversalPosition(double position){
    traversalMotor.set(ControlMode.Position, position);
  }

  public void setTriggerPosition(double position){
    triggerMotor.set(ControlMode.Position, position);
  }

  public boolean ballInTraversal(){
    return traversalBallSensor.objPresent();
  }


  public void armsUp() {
    armsDown = false;
    arms.set(Value.kReverse);
  }

  public void armsDown() {
    //if(!armsDown) armsLoweredTimer = Timer.getFPGATimestamp();
    armsDown = true;
    arms.set(Value.kForward);
  }

  public void armsToggle() {
    arms.toggle();
  }

  public boolean ballInTrigger(){
    return triggerBallSensor.objPresent();
  }

  public boolean getTraversalBallColor(){
    return traversalBallSensor.isRedNotBlue();
  }

  public boolean getTriggerBallColor(){
    return triggerBallSensor.isRedNotBlue();
  }
  
  @Override
  public void periodic() {
    canRunIntake = (armsDown && (Timer.getFPGATimestamp() - armsLoweredTimer) > 0.4);
    SmartDashboard.putNumber("intake current", intakeMotor.getSupplyCurrent());
    SmartDashboard.putBoolean("Arms Down", armsDown);

  }

  public double getIntakePosition() {
    return intakeMotor.getSelectedSensorPosition();
  }

  public double getTraversalPosition() {
    return traversalMotor.getSelectedSensorPosition();
  }

  public void initialize() {
    armsUp();
  }

}
