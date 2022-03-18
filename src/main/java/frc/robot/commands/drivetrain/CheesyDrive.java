/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.CheesyDriveHelper;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class CheesyDrive extends CommandBase {
  /**
   * Creates a new CheesyDrive.
   */
  private final DriveTrain driveTrain;
  private final RobotContainer container;

  private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
  public CheesyDrive(DriveTrain driveTrain, RobotContainer container) {
    this.container = container;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driverJoystick = container.getDriverJoystick();

    if(driverJoystick.getRawButtonPressed(3)) driveTrain.shift(Gear.LOW_GEAR);
    else driveTrain.shift(Gear.HIGH_GEAR);

    double spd = driverJoystick.getY();
    double rot = driverJoystick.getX();

    boolean quickturn = driverJoystick.getRawButton(7);
    driveTrain.arcadeDrive(cheesyDriveHelper.cheesyDrive(spd, rot, quickturn,  driveTrain.getGear() == Gear.HIGH_GEAR)); //todo implement ishighgear
  }
}