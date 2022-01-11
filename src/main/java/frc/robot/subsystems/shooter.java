// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
  private final WPI_TalonSRX shootMotor = new WPI_TalonSRX(Constants.flywheelMotorPort);
  private final WPI_TalonSRX aimMotor = new WPI_TalonSRX(Constants.flywheelAimMotorPort);

  /** Creates a new Shooter. */
  public shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel Speed:", shootMotor.get());
    SmartDashboard.putNumber("Flywheel Aim Speed:", shootMotor.get());
  }

  public void setSpeed(int speed) {
    shootMotor.set(speed);
  }

  public void autoAim() {
    //Vision Logic will go here
    aimMotor.set(1);

  }

}
