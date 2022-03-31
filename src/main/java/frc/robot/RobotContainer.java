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
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.OI.DriverMappings;
import frc.robot.OI.Mappings;
import frc.robot.autonomous.TwoBallAuto;
import frc.robot.commands.climber.JoystickClimber;
import frc.robot.commands.intake.Eject;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeNoTraversal;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.intake.POVIntake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ManualAim;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.commands.shooter.ReverseTrigger;
import frc.robot.commands.shooter.RunTrigger;
import frc.robot.commands.shooter.setpoints.SetpointBase;
import frc.robot.commands.shooter.ProtectedShotSetpoint;
import frc.robot.commands.shooter.LayupShotSetpoint;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.helpers.SetpointHelper;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OldTurret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //declare robot components
  private final Robot robot = new Robot();

  //utilities:
  private final Limelight limelight = new Limelight(2.62, 0.9, 30);
  private final Pigeon pigeon = new Pigeon(new PigeonIMU(20));
  private final PneumaticHub pneumaticHub = new PneumaticHub();
  
  // subsystems:
  private final DriveTrain driveTrain = new DriveTrain();
  private final Flywheel flywheel = new Flywheel();
  private final BallHandler intake = new BallHandler();
  private final OldTurret turret = new OldTurret();
  private final Climber climber = new Climber();
  private final Hood hood = new Hood();
  
  private final Odometry odometry = new Odometry(driveTrain, turret, pigeon, limelight);
  
  // OI:
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick mainOperatorJoystick = new Joystick(1);
  private final Joystick climbOperatorJoystick = new Joystick(2);
  
  private final OI mainOperatorOI = new OI(mainOperatorJoystick);
  private final OI climbOperatorOI = new OI(climbOperatorJoystick);
  private final OI driverOI = new OI(driverJoystick);
  
  // commands:
  private final Command 
    arcadeDrive     = new ArcadeDrive(driveTrain, this),
    joystickAim     = new ManualAim(turret, hood, this),
    povIntake       = new POVIntake(intake, flywheel, this, true),
    protectedShot   = new SetpointBase(flywheel, hood, turret, limelight, 7300, 210, false),
    layupShot       = new SetpointBase(flywheel, hood, turret, limelight, 6200, 60, false),
    insideLine       = new SetpointBase(flywheel, hood, turret, limelight, 6200, 170, false),
    outsideLine       = new SetpointBase(flywheel, hood, turret, limelight, 7200, 220, false),
    manualClimb     = new JoystickClimber(climber, climbOperatorJoystick, driverJoystick, this),
    reverseTrigger  = new ReverseTrigger(flywheel, intake),
    // runTrigger      = new RunTrigger(intake),
    autoShoot       = new AutoShoot(flywheel, hood, turret, intake, limelight, this),
    runIntake       = new Intake(intake).alongWith(new IntakeDown(intake),
                        new RampFlywheel(flywheel).withTimeout(10)
    ),
    runIntakeNoTraversal       = new IntakeNoTraversal(intake).alongWith(new IntakeDown(intake),
                        new RampFlywheel(flywheel).withTimeout(10)
    ),
    runTraversal    = new StartEndCommand(
                        () -> intake.setTraversal(1), 
                        () -> intake.setTraversal(0), 
                        intake
                        ),
    eject = new Eject(intake),
    runTraversalAndTrigger = new StartEndCommand(
      () -> {intake.setTraversal(1); intake.setTrigger(1);}, 
      () -> {intake.setTraversal(0);intake.setTrigger(0);}, 
      intake
      );
      
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      configureButtonBindings();
      limelight.setLedOff();
      limelight.setModeDrive();
    }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(arcadeDrive);
    intake.setDefaultCommand(povIntake);
    hood.setDefaultCommand(joystickAim);
    climber.setDefaultCommand(manualClimb);

    new JoystickButton(mainOperatorJoystick, Mappings.RUN_INTAKE).whenReleased(new IntakeUp(intake).alongWith(new RampFlywheel(flywheel).withTimeout(10)));
    new JoystickButton(driverJoystick, DriverMappings.RUN_INTAKE).whenReleased(new IntakeUp(intake).alongWith(new RampFlywheel(flywheel).withTimeout(10)));

    mainOperatorOI.bind(Mappings.LAYUP_SHOT, layupShot)
              .bind(Mappings.PROTECTED_SHOT, protectedShot)
              .bind(Mappings.INSIDE_LINE_SHOT, insideLine)
              .bind(Mappings.OUTSIDE_LINE_SHOT, outsideLine)
              .bind(Mappings.REVERSE_TRIGGER, reverseTrigger)
              // .bind(Mappings.RUN_TRIGGER, runTrigger)
              .bind(Mappings.EJECT_BALL, eject)
              .bind(Mappings.RUN_INTAKE, runIntake)

              .bind(Mappings.AUTO_SHOOT, autoShoot)
              .bind(Mappings.RUN_TRAVERSAL, runTraversal)
              .bind(Mappings.RUN_TRAVERSAL_AND_TRIGGER, runTraversalAndTrigger);
    
    driverOI.bind(DriverMappings.AUTO_SHOOT, autoShoot)
            .bind(DriverMappings.RUN_INTAKE, runIntake);
  
    //setpoint creation helper
    //testButton1.toggleWhenPressed(new SetpointHelper(flywheel, hood, limelight, testJoystick));//.alongWith(new AutoTurret(turret, odometry.getTarget())));
  }

//   Joystick testJoystick = new Joystick(3);
//   JoystickButton testButton1 = new JoystickButton(testJoystick, 1);
//   public void testModeButtonBindings(){
//  }

  //access functions:
  public Odometry getOdometry() {
    return odometry;
  }

  /**
   * @return the operator joystick
   */
  public Joystick getOperatorJoystick(){
    return mainOperatorJoystick;
  }

  public PneumaticHub getPneumaicsHub(){
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
    return new TwoBallAuto(driveTrain, flywheel, hood, intake, turret, limelight, this);
  }
}
