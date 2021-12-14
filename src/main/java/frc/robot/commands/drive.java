// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;
import edu.wpi.first.wpilibj.Joystick;
public class drive extends CommandBase {
 private final drivetrain m_drivetrain;
 private final Joystick m_stick = new Joystick(0);


  /** Creates a new drive. */
  public drive(drivetrain dt) {
    m_drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.allstop();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_stick.getX();
    double y = m_stick.getY();
    m_drivetrain.arcadedrive(x, y);
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
