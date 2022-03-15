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


  /** Creates a new drive. */
  public ArcadeDrive(DriveTrain driveTrain, RobotContainer container) {
    this.driveTrain = driveTrain;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driverJoystick = container.getDriverJoystick();
    if(driverJoystick.getRawButtonPressed(3)) driveTrain.shift(Gear.LOW_GEAR);
    else driveTrain.shift(Gear.HIGH_GEAR);
    double x = driverJoystick.getX();
    double y = driverJoystick.getY();
    driveTrain.percentDrive(-y - x, -y + x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
