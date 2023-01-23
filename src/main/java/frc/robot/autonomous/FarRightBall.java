// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;

public class FarRightBall extends CommandBase {
  //private final DriveTrain driveTrain;
  @SuppressWarnings("FieldCanBeLocal")
  //private final Flywheel shooter;
  private final DriveTrain driveTrain;
  // private final Hood hood;
  private final BallHandler ballHandler;
  // private final Turret turret;
  // private final Limelight limelight;
  private final RobotContainer container;
  private final Flywheel flywheel;

  private double timer = Timer.getFPGATimestamp();
  private boolean finished = false;

  // private double stage = 0;

  /** Creates a new autonomous. */
  public FarRightBall(DriveTrain driveTrain, Flywheel shooter, Hood hood, BallHandler intake, Turret turret, Limelight limelight, RobotContainer container) {
    this.driveTrain = driveTrain;
    this.flywheel = shooter;
    // this.hood = hood;
    this.ballHandler = intake;
    // this.turret = turret;
    // this.limelight = limelight;
     this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, ballHandler, flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    timer = Timer.getFPGATimestamp();
    container.getPigeon().zeroHeading();
    // while (driveTrain.getAverageEncoderSimpleDistance() != 0) {
     driveTrain.resetEncoders();
    

  }
  private test currentStage = test.BALL1;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double time = Timer.getFPGATimestamp() - timer;

    
    
    SmartDashboard.putNumber("Auto Stage", currentStage.ordinal());    
    switch(currentStage){
      case BALL1:
        driveTrain.percentDrive(0.25, 0.25);
        ballHandler.armsDown();
        ballHandler.setIntake(1);
        ballHandler.setTraversal(0.3);
        if (driveTrain.getAverageEncoderCounts() > 30000){
          currentStage = test.TURN;
          driveTrain.resetEncoders();
        }
        break;
      case TURN:
        driveTrain.percentDrive(-0.2, 0.2);
        if (container.getPigeon().getHeading() > 90) {
          driveTrain.percentDrive(0, 0);
          driveTrain.resetEncoders();
          currentStage = test.BALL2;
        }
        break;

      case BALL2:
        flywheel.flywheelPercent(0.5);
        driveTrain.percentDrive(0.5, 0.5);
        ballHandler.armsDown();
        ballHandler.setTraversal(0.5);
        ballHandler.setIntake(1);
        if (driveTrain.getAverageEncoderCounts() > 70000) {
          currentStage = test.FINISHED;
        }
        break;
      case FINISHED:
        driveTrain.percentDrive(0, 0);
        finished = true;
        break;
      default:
        driveTrain.percentDrive(0, 0);
        finished = true;
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.percentDrive(0, 0);
    ballHandler.armsUp();
    ballHandler.setAll(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  private enum test 
  { 
   BALL1,
   TURN,
   BALL2,
   FINISHED 
  };

}
