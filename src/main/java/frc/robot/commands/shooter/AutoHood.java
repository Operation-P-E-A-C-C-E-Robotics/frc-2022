// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.TargetTracker;
import frc.robot.subsystems.Hood;

public class AutoHood extends CommandBase {
  private final Hood hood;
  private TargetTracker target;

  /** Creates a new AutoAim. */
  public AutoHood(Hood hood, TargetTracker target) {
    this.hood = hood;
    this.target = target;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setHoodForDistance(target.getTargetDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.zero();
  }
}