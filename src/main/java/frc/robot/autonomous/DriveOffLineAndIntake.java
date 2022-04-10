// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.*;

public class DriveOffLineAndIntake extends CommandBase {
  //private final DriveTrain driveTrain;
  @SuppressWarnings("FieldCanBeLocal")
  // private final Flywheel shooter;
  private final DriveTrain driveTrain;
  // private final Hood hood;
  private final BallHandler ballHandler;
  // private final Turret turret;
  // private final Limelight limelight;
  // private final RobotContainer container;

  private double timer = Timer.getFPGATimestamp();
  private boolean isfinished = false;
  /** Creates a new autonomous. */
  public DriveOffLineAndIntake(DriveTrain driveTrain, Flywheel shooter, Hood hood, BallHandler intake, Turret turret, Limelight limelight, RobotContainer container) {
    this.driveTrain = driveTrain;
    // this.shooter = shooter;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp() - timer;
    if(time < 1.1){
      driveTrain.percentDrive(0.5, 0.5);
      ballHandler.armsDown();
      ballHandler.setIntake(1);
      ballHandler.setTraversal(1);
    } else if (time < 1.6){
      ballHandler.armsUp();
      ballHandler.setAll(0);
      driveTrain.percentDrive(-0.5, 0.5);
    } else if (time < 2){
      driveTrain.percentDrive(0, 0);
      // turret.turretRotations(-0.4);
    } else {
      driveTrain.percentDrive(0, 0);
      isfinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }
}
