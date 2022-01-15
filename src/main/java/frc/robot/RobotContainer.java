// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //declare robot components
  //utilities:
  private final Limelight limelight = new Limelight(0, 0);

  // subsystems:
  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  
  // commands:
  private final DrivebaseAutoAim aimCommand = new DrivebaseAutoAim(driveTrain, limelight);
  private final Command autonomous = new Autonomous(driveTrain, shooter);
  
  // OI:
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);
  private final JoystickButton intakeButton = new JoystickButton(operatorJoystick, 2);
  private final JoystickButton traversialButton = new JoystickButton(operatorJoystick, 4);
  private final JoystickButton aimButton = new JoystickButton(operatorJoystick, 6);
  private final JoystickButton shootButton = new JoystickButton(operatorJoystick, 5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //set default commands
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, this));
    shooter.setDefaultCommand(new ManualAim(shooter, this));

    // Configure the button bindings
    configureButtonBindings();
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeButton.whileHeld(new RunIntake(intake));
    aimButton.whileHeld(aimCommand);
    shootButton.whileHeld(new ShooterNoPID(shooter));
  }

  //access functions:
  /**
   * @return the operator joystick
   */
  public Joystick getOperatorJoystick(){
    return operatorJoystick;
  }
  
  /**
   * @return the driver joystick
   */
  public Joystick getDriverJoystick(){
    return driverJoystick;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomous; //todo change to command when written
  }
}
