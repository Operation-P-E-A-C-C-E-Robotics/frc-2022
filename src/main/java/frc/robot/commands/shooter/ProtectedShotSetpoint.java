// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Hood;
import frc.lib.sensors.Limelight;
import frc.robot.subsystems.Flywheel;

public class ProtectedShotSetpoint extends CommandBase {
  private final Flywheel shooter;
  private final Hood hood;
  private double velocity = 8000;

  private double curretTurretPosition, 
  targetTurretPosition = Double.NaN;
  private Limelight limelight;
  private OldTurret turret;


  /** Setpoint for shooting from the protected shot */
  public ProtectedShotSetpoint(Flywheel shooter, Hood hood, OldTurret turret, Limelight camera) {
    this.shooter = shooter;
    this.hood = hood;
    this.limelight = camera;
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(hood);
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
    shooter.flywheelVelocity(velocity);
    hood.setHoodPosition(200);

    if(limelight.hasTarget() == 1){
      curretTurretPosition = turret.getPosition();
      double deltaX = limelight.getTargetOffsetX() / 360;
      double newTargetTurretPosition = curretTurretPosition + deltaX; //todo figure out what's flipped
       

      if(Double.isNaN(targetTurretPosition) || Math.abs(deltaX) > 5){
        targetTurretPosition = newTargetTurretPosition;
      } else {
        targetTurretPosition += (newTargetTurretPosition - targetTurretPosition) / 10;
      }

      turret.setTurretRotations(targetTurretPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheelPercent(0);
    limelight.setModeDrive();
    limelight.setLedOff();
    turret.setTurretPercent(0.0);
    hood.zero();
  }
}
