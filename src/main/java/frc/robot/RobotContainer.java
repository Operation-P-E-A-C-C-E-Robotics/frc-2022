// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.Limelight;
import frc.lib.debloating.Pigeon;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix.sensors.PigeonIMU;

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
  private final Limelight limelight = new Limelight(63, 36, 30);
  private final Pigeon pigeon = new Pigeon(new PigeonIMU(7));
  
  // subsystems:
  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final BallHandler intake = new BallHandler();
  private final Turret turret = new Turret();
  
  private final Odometry odometry = new Odometry(driveTrain, pigeon, limelight);
  
  private final HoodWIthLinearServo hood = new HoodWIthLinearServo();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //set default commands
    
    // Configure the button bindings
    configureButtonBindings();
  }
  
  // commands:
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain, this);
  private final Autonomous autonomous = new Autonomous(driveTrain, shooter);
  private final DrivebaseAutoAim drivebaseAutoAim = new DrivebaseAutoAim(driveTrain, limelight);
  private final ManualTurret joystickAim = new ManualTurret(turret, this);
  private final ManualIntake runIntake = new ManualIntake(intake, this);
  // private final FlywheelPercent runShooterPercent = new FlywheelPercent(shooter);
  private final FlywheelVelocity1 flywheel1 = new FlywheelVelocity1(shooter);
  private final TurretTesting turretTesting = new TurretTesting(turret, this);
  private final LimelightTurret autoAim = new LimelightTurret(turret, limelight);

  // OI:
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);
  private final JoystickButton flywheelButton = new JoystickButton(operatorJoystick, 6);
  private final JoystickButton aimButton = new JoystickButton(operatorJoystick, 8);
  private final JoystickButton zeroButton = new JoystickButton(operatorJoystick, 10);
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(arcadeDrive);
    intake.setDefaultCommand(runIntake);
    // hood.setDefaultCommand(new ManualHoodControl(hood, this));
    turret.setDefaultCommand(joystickAim);

    flywheelButton.whileHeld(flywheel1);
    aimButton.whileHeld(autoAim);
    zeroButton.whenPressed(() -> turret.zero());
  }

  //access functions:
  public Odometry getOdometry() {
    return odometry;
  }
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
