// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShoot extends CommandBase {
  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Limelight limelight;

  private double curretTurretPosition, 
  targetTurretPosition = Double.NaN;
  private BallHandler intake;

  /** Creates a new AutoAim. */
  public AutoShoot(Turret turret, Hood hood, Shooter shooter, BallHandler intake, Limelight limelight) {
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.limelight = limelight;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, hood, shooter);
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

      // hood.setHoodForDistance(limelight.getTargetDistance());
      // hood.zero();
      // hood.setHoodSpeed(0);

      // shooter.setVelcityForDistance(limelight.getTargetDistance());
      shooter.flywheelVelocity(3000);

      if(turret.ready() && shooter.ready()){
        SmartDashboard.putBoolean("run trigger", true);
        intake.setTrigger(1);
        intake.setTraversal(0.5);
      } else{
        SmartDashboard.putBoolean("run trigger", false);
        intake.setTrigger(0);
        intake.setTraversal(0);
      }

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
    hood.zero();
    shooter.flywheelPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}