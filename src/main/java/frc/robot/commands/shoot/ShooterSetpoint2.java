// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ShooterSetpoint2 extends CommandBase {
  private final Shooter shooter;
  private final Hood hood;
  private double veloctiy_default = 6700;
  /** Creates a new ShooterControl. */
  public ShooterSetpoint2(Shooter shooter, Hood hood) {
    this.shooter = shooter;
    this.hood = hood;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("flywheel velocity 1", veloctiy_default);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Replace with PID Control
    // shooter.flywheelVelocity(SmartDashboard.getNumber("shooter velocity", 0));
    shooter.flywheelVelocity(SmartDashboard.getNumber("flywheel velocity 1", veloctiy_default));
    hood.setHoodPosition(32);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheelPercent(0);
    hood.zero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
