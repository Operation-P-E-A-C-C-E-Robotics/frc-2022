// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
//import frc.robot.OI.DriverMappings;
import frc.robot.OI.Mappings;
import frc.robot.autonomous.DriveDistance;
import frc.robot.autonomous.FarHumanPlayerBallAuto;
import frc.robot.autonomous.FarRightBall;
import frc.robot.autonomous.OneBallOffLine;
import frc.robot.autonomous.ThreeBallAuto;
import frc.robot.autonomous.TurnAngle;
import frc.robot.autonomous.TwoBallAuto;
import frc.robot.autonomous.TwoBallRightSide;
import frc.robot.commands.climber.JoystickClimber;
import frc.robot.commands.intake.Eject;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeNoTraversal;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.intake.POVIntake;
import frc.robot.commands.shooter.AutoAim;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ManualAim;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.commands.shooter.ReverseTrigger;
import frc.robot.commands.shooter.RunTrigger;
import frc.robot.commands.shooter.TriggerWhenReady;
import frc.robot.commands.shooter.setpoints.SetpointBase;
import frc.robot.commands.shooter.ProtectedShotSetpoint;
import frc.robot.commands.shooter.LayupShotSetpoint;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.helpers.FlywheelTuner;
import frc.robot.commands.helpers.NewSetpointHelper;
import frc.robot.commands.helpers.SetpointHelper;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //declare robot components
  private final Robot robot = new Robot();

  //utilities: target height 103 inches
  private final Limelight limelight = new Limelight(2.62, 0.9, 20);
  private final Pigeon pigeon = new Pigeon(new PigeonIMU(20));
  private final PneumaticHub pneumaticHub = new PneumaticHub();
  
  // subsystems:
  private final DriveTrain driveTrain = new DriveTrain();
  private final Flywheel flywheel = new Flywheel();
  private final BallHandler intake = new BallHandler();
  private final Turret turret = new Turret();
  private final Climber climber = new Climber();
  private final Hood hood = new Hood();
  
  
  
  // OI:
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick mainOperatorJoystick = new Joystick(1);
  private final Joystick climbOperatorJoystick = new Joystick(2);
  
  private final OI mainOperatorOI = new OI(mainOperatorJoystick);
  private final OI climbOperatorOI = new OI(climbOperatorJoystick);
  //private final OI driverOI = new OI(driverJoystick);
  
  // commands:
  private final Command 
    arcadeDrive     = new ArcadeDrive(driveTrain, this),
    joystickAim     = new ManualAim(turret, hood, this),
    povIntake       = new POVIntake(intake, flywheel, this, true),
    protectedShot   = new SetpointBase(flywheel, hood, turret, limelight, this, 7600, 250, false),
    layupShot       = new SetpointBase(flywheel, hood, turret, limelight, this, 6200, 60, false),
    insideLine       = new SetpointBase(flywheel, hood, turret, limelight, this, 3200, 270, false),
    outsideLine       = new SetpointBase(flywheel, hood, turret, limelight, this, 7130, 220, false),
    manualClimb     = new JoystickClimber(climber, climbOperatorJoystick, driverJoystick, this),
    reverseTrigger  = new ReverseTrigger(flywheel, intake),
    // runTrigger      = new RunTrigger(intake),
    autoAim = new AutoAim(flywheel, hood, turret, limelight, this),
    autoShoot       = new AutoShoot(flywheel, hood, turret, intake, driveTrain, limelight, this),
    autoTrigger     = new TriggerWhenReady(turret, hood, flywheel, intake, driveTrain, limelight),
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
      driveTrain.resetEncoders();


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
    //new JoystickButton(driverJoystick, DriverMappings.RUN_INTAKE).whenReleased(new IntakeUp(intake).alongWith(new RampFlywheel(flywheel).withTimeout(10)));


      // Add commands to the autonomous command chooser
      m_chooser.addOption("One Ball", OneBallLine);
      m_chooser.setDefaultOption("Two Ball", TwoBallLeft);
      m_chooser.addOption("Two Ball Right", TwoBallRight);
      m_chooser.addOption("Three Ball", ThreeBall);

    SmartDashboard.putData(m_chooser);

    mainOperatorOI.bind(Mappings.LAYUP_SHOT, layupShot)
              .bind(Mappings.PROTECTED_SHOT, protectedShot)
              .bind(Mappings.INSIDE_LINE_SHOT, insideLine)
              .bind(Mappings.OUTSIDE_LINE_SHOT, outsideLine)
              .bind(Mappings.REVERSE_TRIGGER, reverseTrigger)
              // .bind(Mappings.RUN_TRIGGER, runTrigger)
              .bind(Mappings.EJECT_BALL, eject)
              .bind(Mappings.RUN_INTAKE, runIntakeNoTraversal)

              .bind(Mappings.AUTO_SHOOT, autoTrigger)
              .bind(Mappings.RUN_TRAVERSAL, runTraversal)
              .bind(Mappings.PREPARE_SHOOT, autoAim)
              .bind(Mappings.RUN_TRAVERSAL_AND_TRIGGER, runTraversalAndTrigger);
    
    // driverOI.bind(DriverMappings.AUTO_SHOOT, autoShoot)
    //         .bind(DriverMappings.RUN_INTAKE, runIntake);
  
    //setpoint creation helper
    //testButton1.toggleWhenPressed(new NewSetpointHelper(flywheel, hood, limelight, testJoystick));//.alongWith(new AutoTurret(turret, odometry.getTarget())));
    //testButton2.toggleWhenPressed(new FlywheelTuner(flywheel, testJoystick));
  }

  Joystick testJoystick = new Joystick(3);
  //JoystickButton testButton1 = new JoystickButton(testJoystick, 1);
  //JoystickButton testButton2 = new JoystickButton(testJoystick, 2);
  //   public void testModeButtonBindings(){
//  }
private final Odometry odometry = new Odometry(driveTrain, turret, pigeon, limelight, testJoystick);

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

  public Pigeon getPigeon() {
    return pigeon;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {  
    return m_chooser.getSelected();  
    // return new ThreeBallAuto(driveTrain, intake, flywheel, turret, hood, limelight, this, pigeon);
    //new TwoBallAutoTwoBallLeft(driveTrain, flywheel, hood, intake, turret, limelight, this);
  }

  private Command TwoBallLeft = new TwoBallAuto(driveTrain, flywheel, hood, intake, turret, limelight, this);

  // private Command ThreeBall = new FarHumanPlayerBallAuto(driveTrain, flywheel, hood, intake, turret, limelight, this);
  private Command ThreeBall = new ThreeBallAuto(driveTrain, intake, flywheel, turret, hood, limelight, this, pigeon);

  private Command TwoBallRight = new TwoBallRightSide(driveTrain, pigeon, flywheel, hood, turret, intake, limelight, this);

  private Command OneBallLine = new OneBallOffLine(driveTrain, flywheel, hood, turret, intake, limelight, pigeon, this);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
}

