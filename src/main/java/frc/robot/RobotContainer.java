// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.Limelight;
import frc.lib.debloating.Pigeon;
import frc.robot.autonomous.RealAuto;
import frc.robot.commands.climber.JoystickClimber;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.POVIntake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ManualAim;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.commands.shooter.ReverseTrigger;
import frc.robot.commands.shooter.RunTrigger;
import frc.robot.commands.shooter.ShooterSetpoint1;
import frc.robot.commands.shooter.ShooterSetpoint2;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;
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
  private final Pigeon pigeon = new Pigeon(new PigeonIMU(20));
  private final PneumaticHub pneumaticHub = new PneumaticHub();
  private Robot robot = new Robot();
  // subsystems:
  private final DriveTrain driveTrain = new DriveTrain();
  private final Flywheel shooter = new Flywheel();
  private final BallHandler intake = new BallHandler();
  private final Turret turret = new Turret();
  private final Climber climber = new Climber();

  private final Odometry odometry = new Odometry(driveTrain, turret, pigeon, limelight);
  
  private final Hood hood = new Hood();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //set default commands
    
    // Configure the button bindings
    configureButtonBindings();
  }
  
  // commands:
  private final Command 
    arcadeDrive  = new ArcadeDrive(driveTrain, this),
    joystickAim  = new ManualAim(turret, hood, this),
    povIntake    = new POVIntake(intake, shooter, this, true),
    flywheel1    = new ShooterSetpoint1(shooter, hood, turret, limelight),
    flywheel2    = new ShooterSetpoint2(shooter, hood),
    manualClimb  = new JoystickClimber(climber, this);
  
  // OI:
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);

  private final JoystickButton 
    autoshootButton   = new JoystickButton(driverJoystick, 2), 
    layupShot         = new JoystickButton(operatorJoystick, 1), 
    protectedShot     = new JoystickButton(operatorJoystick, 2), 
    reverseTrigger    = new JoystickButton(operatorJoystick, 3), 
    triggerButton     = new JoystickButton(operatorJoystick, 4), 
    intakeButton      = new JoystickButton(operatorJoystick, 5), 
    autoshootButton2  = new JoystickButton(operatorJoystick, 6), 
    traversalButton   = new JoystickButton(operatorJoystick, 7), 
    travtrigButton     = new JoystickButton(operatorJoystick, 8);
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(arcadeDrive);
    intake.setDefaultCommand(povIntake);
    turret.setDefaultCommand(joystickAim);
    hood.setDefaultCommand(joystickAim);
    climber.setDefaultCommand(manualClimb);
  
    autoshootButton.whileHeld(new AutoShoot(shooter, hood, turret, intake, limelight, this));
    autoshootButton2.whileHeld(new AutoShoot(shooter, hood, turret, intake, limelight, this));

    layupShot.whileHeld(flywheel2);
    protectedShot.whileHeld(flywheel1);
    reverseTrigger.whileHeld(new ReverseTrigger(shooter, intake));
    triggerButton.whileHeld(new RunTrigger(intake));
    intakeButton.whileHeld(
      new Intake(intake).alongWith(
      new IntakeDown(intake),
      new RampFlywheel(shooter).withTimeout(10)));
    traversalButton.whileHeld(new StartEndCommand(
      () -> intake.setTraversal(1), 
      () -> intake.setTraversal(0), 
      intake));
    triggerButton.whileHeld(new StartEndCommand(
      () -> intake.setTrigger(1), 
      () -> intake.setTrigger(0), 
      intake));
    
    travtrigButton.whileHeld(new StartEndCommand(
      () -> {intake.setTraversal(1); intake.setTrigger(1);}, 
      () -> {intake.setTraversal(0);intake.setTrigger(0);}, 
      intake));
    // new JoystickButton(operatorJoystick, 3).whileHeld(() -> {hood.setEncoderZero();});
    // new JoystickButton(operatorJoystick, 4).whileHeld(new HoodTesting(hood, limelight, this));
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

  public Robot getRobot(){
    return robot;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // try {
      return new RealAuto(driveTrain, shooter, hood, intake, turret, limelight, this);
    // } catch (IOException e) {
    //   e.printStackTrace();
    //   return null;
    // } //todo change to command when written
  }
}
