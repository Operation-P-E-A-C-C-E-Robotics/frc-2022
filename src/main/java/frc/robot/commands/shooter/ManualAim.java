// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.NiceCurve;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class ManualAim extends CommandBase {
  private final Turret turret;
  private final RobotContainer container;
  private final NiceCurve curve = NiceCurve.preset1();

  // private double turretRotations = 0;
  private Hood hood;

  // private double hoodCounts = 0;

  private double timer = 0;
  boolean hasControl = false;
  /** Aim with joystick. */
  public ManualAim(Turret shooter, Hood hood, RobotContainer container) {
    this.turret = shooter;
    this.hood = hood;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // turretRotations = turret.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(container.getOperatorJoystick().getRawButton(3)){
    //   turret.zero();
    // }
    if(Math.abs(container.getOperatorJoystick().getX()) > 0.2) {
      // turretRotations += curve.get(container.getOperatorJoystick().getX()) * 0.1;
      // turret.turretRotations(turretRotations);
      turret.setTurretPercent(curve.get(container.getOperatorJoystick().getX()));
    }
    else {
      turret.setTurretPercent(0.0);
      // turretRotations = turret.getPosition();
      // hoodCounts = hood.getHoodPosition();
    }

    if(Math.abs(container.getOperatorJoystick().getRawAxis(2)) > 0.2){
      // hood.setHoodSpeed(-container.getOperatorJoystick().getY() / 3);
      timer = Timer.getFPGATimestamp();  
      hasControl = true;
    } else if ((Timer.getFPGATimestamp() - timer) > 1){
      hasControl = false;
    }
    if (hasControl) {
      hood.setHoodPercent(-container.getOperatorJoystick().getRawAxis(2) / 2);
    }
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}

