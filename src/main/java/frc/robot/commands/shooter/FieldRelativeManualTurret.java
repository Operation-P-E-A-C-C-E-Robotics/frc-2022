// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.NiceCurve;
import frc.lib.math.PointTracker;
import frc.lib.sensors.Pigeon;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Turret;

public class FieldRelativeManualTurret extends CommandBase {
  private final Turret turret;
  private final RobotContainer container;
  private final Hood hood;
  
  private double turretRotations = 0;
  private double hoodCounts = 0;
  private double timer = 0;
  boolean hasControl = false;
  
  private final NiceCurve curve = NiceCurve.preset1();

  private final double fieldHeading;
  /** Aim with joystick. */
    private Pigeon pigeon;
  public FieldRelativeManualTurret(Turret shooter, Hood hood, Pigeon pigeon, RobotContainer container, double fieldHeading) {
    this.turret = shooter;
    this.hood = hood;
    this.pigeon = pigeon;
    this.container = container;

    this.fieldHeading = fieldHeading;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      turretRotations = turret.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotHeading = -((pigeon.getYaw() - fieldHeading) / 360);
    double y = -container.getOperatorJoystick().getRawAxis(0);
    double x = container.getOperatorJoystick().getRawAxis(1);
    PointTracker point = new PointTracker(1);
    point.xy(x,y);

    if(point.r() > 0.3){
        double angle = (point.p() / (2 * Math.PI));
        SmartDashboard.putNumber("joystick angle", angle);
        SmartDashboard.putNumber("robot heading", robotHeading);
        turret.setTurretRotations((((angle - robotHeading) + 0.5) % 1) - 0.5);
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

