// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_CONTROLLER_PORT);
  
  /** Creates a new Intake. */
  public Intake() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake current", intakeMotor.getSupplyCurrent());
  }

  public void setPercent(double speed){
    System.out.println(speed);
    intakeMotor.set(speed);
  }
}
