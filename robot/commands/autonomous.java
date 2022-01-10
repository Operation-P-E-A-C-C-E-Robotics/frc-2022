// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.shooter;

public class autonomous extends CommandBase {
  private final drivetrain driveTrain;
  private static shooter shooter;
  
  /** Creates a new autonomous. */
  public autonomous(drivetrain dt) {
    driveTrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Aims the shooter and fires the preloaded ball then backs out of the tarmac
    shooter.autonomousAim();
    shooter.setSpeed(1);

    driveTrain.tankDrive(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
