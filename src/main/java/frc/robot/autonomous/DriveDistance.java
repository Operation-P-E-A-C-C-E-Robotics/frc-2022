// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class DriveDistance extends CommandBase {
  //private final DriveTrain driveTrain;
  @SuppressWarnings("FieldCanBeLocal")
  //private final Flywheel shooter;
  private final DriveTrain driveTrain;
  private final RobotContainer container;
private double distance;
private double rate;

  /** Creates a new autonomous. */
  public DriveDistance(DriveTrain driveTrain, RobotContainer container, double distance, double rate) {
    this.driveTrain = driveTrain;
     this.container = container;
     this.distance = distance;
     this.rate = rate;
     System.out.println(":):)::):):):):):)");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      driveTrain.shift(Gear.HIGH_GEAR);
     driveTrain.resetEncoders();
     System.out.println(":):):):):):):):):):):):):):):):):):):):):)");
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {    
        driveTrain.percentDrive(rate, rate);
        
        System.out.println(driveTrain.getAverageEncoderMeters());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrain.shift(Gear.LOW_GEAR);
      driveTrain.percentDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveTrain.getAverageEncoderMeters()) > distance;
  }

}
