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
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.*;

public class FarRightBall extends CommandBase {
  //private final DriveTrain driveTrain;
  @SuppressWarnings("FieldCanBeLocal")
  //private final Flywheel shooter;
  private final DriveTrain driveTrain;
  // private final Hood hood;
  private final BallHandler ballHandler;
  // private final Turret turret;
  // private final Limelight limelight;
  // private final RobotContainer container;

  private double timer = Timer.getFPGATimestamp();
  private boolean isfinished = false;

  private double stage = 0;

  /** Creates a new autonomous. */
  public FarRightBall(DriveTrain driveTrain, Flywheel shooter, Hood hood, BallHandler intake, OldTurret turret, Limelight limelight, RobotContainer container) {
    this.driveTrain = driveTrain;
     
    // this.hood = hood;
    this.ballHandler = intake;
    // this.turret = turret;
    // this.limelight = limelight;
    // this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, ballHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = Timer.getFPGATimestamp();
    
    while (driveTrain.getAverageEncoderSimpleDistance() != 0) {
     driveTrain.resetEncoders();
    }

  }
  private test currentStage = test.BALL1;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp() - timer;

    
    
    // SmartDashboard.putNumber("Auto Stage", stage);    

    // if(stage == 0){
    //   driveTrain.percentDrive(0.5, 0.5);
    //   ballHandler.armsDown();
    //   ballHandler.setIntake(1);
    //   ballHandler.setTraversal(0.5);
    //   stage = 1;
    // } 
    // if (stage == 1){
    //   Timer.delay(1.5);
    //   ballHandler.armsUp();
    //   ballHandler.setIntake(0);
    //   ballHandler.setTraversal(0);
    //   driveTrain.percentDrive(0, 0);
    //   stage = 2;
    // } 
    // if (stage == 2) {
    //   driveTrain.percentDrive(-0.5, 0.5);
    //   Timer.delay(1.75);
    //   stage = 3;
    // }
    // if (stage == 3){
    //   driveTrain.percentDrive(0, 0);
    //   // turret.turretRotations(-0.4);
    // } else {
    //   driveTrain.percentDrive(0, 0);
    //   isfinished = true;
    // }
    switch(currentStage){
      case BALL1:
        driveTrain.percentDrive(0.25, 0.25);
        ballHandler.armsDown();
        ballHandler.setIntake(1);
        ballHandler.setTraversal(1);
        if (driveTrain.getAverageEncoderSimpleDistance() < 31000){
          currentStage = test.TURN;
          driveTrain.resetEncoders();
        }
        break;
      case TURN:
      driveTrain.percentDrive(-0.25, 0.25);
      if (driveTrain.getLeftEncoderSimple() < 31000){
        currentStage = test.BALL2;
        driveTrain.resetEncoders();
       }
        break;

      case BALL2:
        break;
      default:
        break;
    }

    if(driveTrain.getAverageEncoderSimpleDistance() < 31000){
      driveTrain.percentDrive(0.25, 0.25);
      ballHandler.armsDown();
      ballHandler.setIntake(1);
      ballHandler.setTraversal(1);
    // } else if (time <  1.56){
    //   driveTrain.percentDrive(-0.5, 0.5);


  
      

    // } else if (time < 2.5){
    //   driveTrain.percentDrive(0, 0);
    //   ballHandler.armsUp();
    //   ballHandler.setAll(0);
    //   // turret.turretRotations(-0.4);
    // } else if (time < 3.55 ) {
    //   driveTrain.percentDrive(0.5, 0.5);
      
    } else {
      isfinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.percentDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }

  private enum test 
  { 
   BALL1,
   TURN,
   BALL2 
  };

}
