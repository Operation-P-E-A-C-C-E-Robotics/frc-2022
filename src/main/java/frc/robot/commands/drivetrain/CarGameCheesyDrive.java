package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.CheesyDriveHelper;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

//i made this to play with github copilot :)
//thank you ai for this nice code.

public class CarGameCheesyDrive extends CommandBase {
  /**
   * Creates a new CheesyDrive.
   */
  private final DriveTrain driveTrain;
  private final RobotContainer container;

  private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

  /**
   * a cheesy drive command that uses similar controls to a driving game.
   * The left trigger will make the robot go backwards, the right trigger will make the robot go forwards,
   * and the right joystick controls the rotation.
   * @param driveTrain the drive train subsystem
   * @param container the robot container
   */
  public CarGameCheesyDrive(DriveTrain driveTrain, RobotContainer container) {
    this.container = container;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driverJoystick = container.getDriverJoystick();

    if(driverJoystick.getRawButtonPressed(2)) driveTrain.shift(DriveTrain.Gear.LOW_GEAR);
    else driveTrain.shift(DriveTrain.Gear.HIGH_GEAR);

    double leftTrigger = driverJoystick.getRawAxis(2);
    double rightTrigger = driverJoystick.getRawAxis(3);
    double spd = -leftTrigger + rightTrigger;
    double rot = driverJoystick.getX();

    boolean quickturn = driverJoystick.getRawButton(4);
    driveTrain.arcadeDrive(cheesyDriveHelper.cheesyDrive(spd, rot, quickturn,  driveTrain.getGear() == DriveTrain.Gear.HIGH_GEAR)); //todo implement ishighgear
  }
}
