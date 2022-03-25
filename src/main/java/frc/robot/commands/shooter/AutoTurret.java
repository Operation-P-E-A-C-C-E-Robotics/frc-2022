// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.robot.subsystems.Turret;

public class AutoTurret extends CommandBase {
  private final Turret turret;
  private final Limelight limelight;

  private double curretTurretPosition, 
  targetTurretPosition = Double.NaN;

  private boolean smoothing = false;
  // private TargetTracker target;

  /** Creates a new AutoAim. */
  public AutoTurret(Turret turret, Limelight limelight) {
    this.turret = turret;
    // this.target = target;
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
    smoothing = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1){
      curretTurretPosition = turret.getPosition();
      double deltaX = limelight.getTargetOffsetX() / 360;
      double newTargetTurretPosition = curretTurretPosition + deltaX; //todo figure out what's flipped
      // double newTargetTurretPosition = target.getTargetAngle();
      if(deltaX < 0.05) smoothing = true;

      if(Double.isNaN(targetTurretPosition) || !smoothing){
        targetTurretPosition = newTargetTurretPosition;
      } else {
        targetTurretPosition += (newTargetTurretPosition - targetTurretPosition) / 10;
      }
      

      turret.setTurretRotations(newTargetTurretPosition);
    } else{
      targetTurretPosition = Double.NaN;
      turret.setTurretPercent(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetTurretPosition = Double.NaN;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
