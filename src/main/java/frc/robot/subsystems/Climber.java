// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
  private WPI_TalonFX climberTop = new WPI_TalonFX(CLIMBER_TOP_CONTROLLER_PORT); 
  private WPI_TalonFX climberBottom = new WPI_TalonFX(CLIMBER_BOTTOM_CONTROLLER_PORT);
  private DoubleSolenoid iNeedAName1 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 5, 6);//TODO Name Me
  private DoubleSolenoid iNeedAName2 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 7, 8);//TODO Name Me
  
  /** Creates a new Climber. */
  public Climber() {}
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
