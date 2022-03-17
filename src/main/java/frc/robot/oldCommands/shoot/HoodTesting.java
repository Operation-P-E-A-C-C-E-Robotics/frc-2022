// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oldCommands.shoot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodTesting extends CommandBase {
  private Hood hood;
  private RobotContainer container;
  private Limelight limelight;
  /** Creates a new ManualHoodAdjust. */
  public HoodTesting(Hood hood, Limelight limelight, RobotContainer container) {
    this.hood = hood;
    this.limelight = limelight;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setModeVision();
    limelight.setLedOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setHoodAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood setpoint", 0)));
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
