// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.Limelight;
import frc.lib.debloating.Pigeon;
import frc.robot.commands.auto.paths.PathBase;
import frc.robot.commands.auto.paths.TestPath;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.intake.ManualIntake;
import frc.robot.commands.shoot.AutoShoot;
import frc.robot.commands.shoot.FlywheelVelocity1;
import frc.robot.commands.shoot.HoodTesting;
import frc.robot.commands.shoot.LimelightTurret;
import frc.robot.commands.shoot.ManualHood;
import frc.robot.commands.shoot.ManualTrigger;
import frc.robot.commands.shoot.ManualTurret;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //declare robot components
  //utilities:
  private final Limelight limelight = new Limelight(2.62, 0.9, 30);
  private final Pigeon pigeon = new Pigeon(new PigeonIMU(7));
  private final PneumaticHub pneumaticHub = new PneumaticHub();
  // subsystems:
  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final BallHandler intake = new BallHandler();
  private final Turret turret = new Turret();
  
  private final Odometry odometry = new Odometry(driveTrain, pigeon, limelight);
  
  private final Hood hood = new Hood();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //set default commands
    
    // Configure the button bindings
    configureButtonBindings();
  }
  
  // commands:
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain, this);
  private final ManualTurret joystickAim = new ManualTurret(turret, this);
  private final ManualIntake runIntake = new ManualIntake(intake, this);
  private final FlywheelVelocity1 flywheel1 = new FlywheelVelocity1(shooter);
  private final LimelightTurret autoAim = new LimelightTurret(turret, limelight);
  private final ManualHood manualHood = new ManualHood(hood, this);

  // OI:
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);
  private final JoystickButton autoshootButton = new JoystickButton(operatorJoystick, 6);
  private final JoystickButton triggerButton = new JoystickButton(operatorJoystick, 8);
  private final JoystickButton velocity1JoystickButton = new JoystickButton(operatorJoystick, 1);
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(arcadeDrive);
    intake.setDefaultCommand(runIntake);
    turret.setDefaultCommand(joystickAim);
    hood.setDefaultCommand(manualHood);
  
    autoshootButton.whileHeld(new AutoShoot(turret, hood, shooter, intake, limelight));
    triggerButton.whileHeld(new ManualTrigger(intake));
    velocity1JoystickButton.whileHeld(flywheel1);
    new JoystickButton(operatorJoystick, 3).whileHeld(new HoodTesting(hood, limelight, this));
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

  public PneumaticHub gPneumaticHub(){
    return pneumaticHub;
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
  public PathBase getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    try {
      return new TestPath(driveTrain, odometry);
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    } //todo change to command when written
  }
}
