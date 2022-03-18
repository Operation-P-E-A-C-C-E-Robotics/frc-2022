// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;

public class LayupShotSetpoint extends CommandBase {
  private final Flywheel shooter;
  private final Hood hood;
  private double veloctiy = 6700;
  
  /** Creates a new ShooterControl. */
  public LayupShotSetpoint(Flywheel shooter, Hood hood) {
    this.shooter = shooter;
    this.hood = hood;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(hood);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Replace with PID Control
    shooter.flywheelVelocity(veloctiy);
    hood.setHoodPosition(32);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheelPercent(0);
    hood.zero();
  }
}
