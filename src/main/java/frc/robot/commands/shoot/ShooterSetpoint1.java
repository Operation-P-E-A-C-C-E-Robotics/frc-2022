// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ShooterSetpoint1 extends CommandBase {
  private final Shooter shooter;
  private final Hood hood;
  private double veloctiy_default = 8000;

  private double curretTurretPosition, 
  targetTurretPosition = Double.NaN;
  private Limelight limelight;
  private Turret turret;


  /** Creates a new ShooterControl. */
  public ShooterSetpoint1(Shooter shooter, Hood hood, Turret turret, Limelight camera) {
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
    SmartDashboard.putNumber("flywheel velocity 1", veloctiy_default);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Replace with PID Control
    // shooter.flywheelVelocity(SmartDashboard.getNumber("shooter velocity", 0));
    shooter.flywheelVelocity(SmartDashboard.getNumber("flywheel velocity 1", veloctiy_default));
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

      turret.turretRotations(targetTurretPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheelPercent(0);
    limelight.setModeDrive();
    limelight.setLedOff();
    turret.turretPercent(0.0);
    hood.zero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
