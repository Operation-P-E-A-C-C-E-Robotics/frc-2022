// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class LimelightTurret extends CommandBase {
  private final Turret turret;
  private final Limelight limelight;

  private double curretTurretPosition, 
  targetTurretPosition = Double.NaN;

  /** Creates a new AutoAim. */
  public LimelightTurret(Turret turret, Limelight limelight) {
    this.turret = turret;
    this.limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setModeVision();
      limelight.setLedOn();
      targetTurretPosition = Double.NaN;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1){
      curretTurretPosition = turret.getPosition();
      double deltaX = limelight.getTargetOffsetX() / 360;
      double newTargetTurretPosition = curretTurretPosition + deltaX; //todo figure out what's flipped

      if(Double.isNaN(targetTurretPosition) || Math.abs(deltaX) > 5){
        targetTurretPosition = newTargetTurretPosition;
      } else {
        targetTurretPosition += (newTargetTurretPosition - targetTurretPosition) / 10;
      }


      turret.turretRotations(targetTurretPosition);

      SmartDashboard.putNumber("dist to target", limelight.getTargetDistance());
    } else{
      targetTurretPosition = Double.NaN;
      turret.turretPercent(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetTurretPosition = Double.NaN;
    limelight.setModeDrive();
    limelight.setLedOff();
    turret.turretPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
