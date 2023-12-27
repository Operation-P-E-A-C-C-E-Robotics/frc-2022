// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import edu.wpi.first.wpilibj.Joystick;

public class ArcadeDrive extends CommandBase {
 private final DriveTrain driveTrain;
  private final RobotContainer container;

  private final double velocityMultiplier = 3000;

  /** Creates a new drive. */
  public ArcadeDrive(DriveTrain driveTrain, RobotContainer container) {
    this.driveTrain = driveTrain;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driverJoystick = container.getDriverJoystick();

    if(driverJoystick.getRawButton(2)) driveTrain.shift(Gear.LOW_GEAR);
    else driveTrain.shift(Gear.HIGH_GEAR);

    double x = driverJoystick.getX();
    double y = driverJoystick.getY();
    
    double left = -y - x;
    double right = -y + x;

    driveTrain.percentDrive(left, right);
    // driveTrain.velocityDrive(left * velocityMultiplier * driverJoystick.getThrottle(), right * velocityMultiplier * driverJoystick.getThrottle());
  }
}
