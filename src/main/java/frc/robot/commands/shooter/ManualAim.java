// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.NiceCurve;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class ManualAim extends CommandBase {
  private final Turret turret;
  private final RobotContainer container;
  private final NiceCurve curve = NiceCurve.preset1();

  private double turretRotations = 0;
  /** Creates a new manualShooter. */
  public ManualAim(Turret shooter, RobotContainer container) {
    this.turret = shooter;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      turretRotations = turret.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(container.getOperatorJoystick().getRawButton(3)){
    //   turret.zero();
    // }
    if(Math.abs(container.getOperatorJoystick().getX()) > 0.1) {
      turretRotations += curve.get(container.getOperatorJoystick().getX()) * 0.4;
      turret.turretRotations(turretRotations);
    }
    else {
      turret.turretPercent(0.0);
      turretRotations = turret.getPosition();
    }
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

