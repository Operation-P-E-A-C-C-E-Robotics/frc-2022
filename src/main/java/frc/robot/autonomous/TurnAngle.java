// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Pigeon;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class TurnAngle extends CommandBase {
  //private final DriveTrain driveTrain;
  @SuppressWarnings("FieldCanBeLocal")
  //private final Flywheel shooter;
  private final DriveTrain driveTrain;
  private final RobotContainer container;
  private final double kP = 0;
private Rotation2d angle;
    private double lSpeed, rSpeed;
    private Pigeon pigeon;

  /** Creates a new autonomous. */
  public TurnAngle(DriveTrain driveTrain, Pigeon pigeon, RobotContainer container, Rotation2d angle, double rate) {
    this.driveTrain = driveTrain;
    this.pigeon = pigeon;
     this.container = container;
     this.angle = angle;
     boolean clockwise = angle.getDegrees() > 0;
     lSpeed = clockwise ? rate : -rate;
     rSpeed = clockwise ? -rate : rate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    container.getPigeon().zeroHeading();
    driveTrain.shift(Gear.HIGH_GEAR);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("r asdfjkl", rSpeed);
    // SmartDashboard.putNumber("l asdfjkl", lSpeed);
//     driveTrain.percentDrive(
//       rSpeed * kP * Math.abs(angle.getDegrees() - pigeon.getHeading()), 
//       rSpeed * kP * Math.abs(angle.getDegrees() - pigeon.getHeading())
//     );
    driveTrain.percentDrive(rSpeed, lSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveTrain.percentDrive(0, 0);
      driveTrain.shift(Gear.LOW_GEAR);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
      Math.abs(pigeon.getRelativeHeading()) > Math.abs(angle.getDegrees()));
  }

}
