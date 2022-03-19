// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BallHandlerConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallHandler extends SubsystemBase {
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_CONTROLLER_PORT);
  private final WPI_TalonSRX traversalMotor = new WPI_TalonSRX(TRAVERSAL_CONTROLLER_PORT);
  private final WPI_TalonSRX triggerMotor = new WPI_TalonSRX(TRIGGER_CONTROLLER_PORT); //todo do port number in constants

  private boolean canRunIntake = false,
                  armsDown = false;
  private double armsLoweredTimer = 0;


  //Arm Pnuematics
  private final DoubleSolenoid arms = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 2, 3);

  /** Creates a new Intake. */
  public BallHandler() {
    traversalMotor.setInverted(true);
    intakeMotor.setInverted(true);
    traversalMotor.configContinuousCurrentLimit(5);
    armsUp();
  }

  /**
   * set the intake percent
   * @param speed percent (-1 to 1), positive forward
   */
  public void setIntake(double speed){
    if (canRunIntake) intakeMotor.set(speed);
    else intakeMotor.set(0);
  }

  /**
   * set the traversal percent
   * @param speed percent (-1 to 1), positive forward
   */
  public void setTraversal(double speed){
    traversalMotor.set(speed);
  }

  /**
   * set the trigger percent
   * @param speed percent (-1 to 1), positive forward
   */
  public void setTrigger(double speed){
    triggerMotor.set(speed);
  }

  /**
   * set intake, traversal, and trigger to the same speed
   * @param speed percent (-1 to 1)
   */
  public void setAll(double speed){
    setIntake(speed);
    setTraversal(speed);
    setTrigger(speed);
  }

  /**
   * raise intake arms
   */
  public void armsUp() {
    armsDown = false;
    arms.set(Value.kReverse);
  }

  /**
   * lower intake arms
   */
  public void armsDown() {
    if(!armsDown) armsLoweredTimer = Timer.getFPGATimestamp();
    armsDown = true;
    arms.set(Value.kForward);
  }
  
  @Override
  public void periodic() {
    //only allow the intake to run if the arms have had 0.4 secs to lower
    canRunIntake = (armsDown && (Timer.getFPGATimestamp() - armsLoweredTimer) > 0.4);
  }
}
