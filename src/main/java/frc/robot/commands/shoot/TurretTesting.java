// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretTesting extends CommandBase {
  private final Turret turret;
  private final RobotContainer containter;
  /** Creates a new manualShooter. */
  public TurretTesting(Turret shooter, RobotContainer container) {
    this.turret = shooter;
    this.containter = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    SmartDashboard.putNumber("turret target rotations", 0);
    // SmartDashboard.putNumber("turret p", 0);
    // SmartDashboard.putNumber("turret i", 0);
    // SmartDashboard.putNumber("turret d", 0);
    // SmartDashboard.putNumber("turret f", 0);
    // SmartDashboard.putNumber("turret iz", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //   turret.getGainSetter()
    //   .ff(SmartDashboard.getNumber("turret f", 0))
    //   .p(SmartDashboard.getNumber("turret p", 0))
    //   .i(SmartDashboard.getNumber("turret i", 0))
    //   .d(SmartDashboard.getNumber("turret d", 0))
    //   .iz(SmartDashboard.getNumber("turret iz", 0));
    turret.turretRotations(SmartDashboard.getNumber("turret target rotations", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      turret.turretPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}