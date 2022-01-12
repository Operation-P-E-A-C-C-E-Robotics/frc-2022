// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class Autonomous extends CommandBase {
  private final DriveTrain driveTrain;
  private final Shooter shooter;
  /** Creates a new autonomous. */
  public Autonomous(DriveTrain dt, Shooter shr) {
    driveTrain = dt;
    shooter = shr;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.autoAim();
    shooter.setSpeed(1);
    driveTrain.tankDrive(0.5, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
