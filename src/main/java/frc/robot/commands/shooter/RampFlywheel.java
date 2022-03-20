// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class RampFlywheel extends CommandBase {

  private Flywheel flywheel;

/** Creates a new rev the flywheel to a fixed percentage. */
  public RampFlywheel(Flywheel flywheel) {
      this.flywheel = flywheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      flywheel.flywheelPercent(0.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      flywheel.flywheelPercent(0);
  }
}
