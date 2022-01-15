// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_CONTROLLER_PORT);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercent(double speed){
    intakeMotor.set(speed);
  }
}
