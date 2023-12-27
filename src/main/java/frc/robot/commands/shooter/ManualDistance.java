// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;

public class ManualDistance extends CommandBase {
  private final Hood hood;
  private Limelight target;
private Flywheel shooter;

    double distance = 0;
    private RobotContainer container;

  /** Creates a new AutoAim. */
  public ManualDistance(Hood hood, Flywheel shooter, RobotContainer container) {
    this.hood = hood;
    this.shooter = shooter;
    this.container = container;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      distance += container.testJoystick.getY()/100;
    hood.setHoodForDistance(distance);
    shooter.setVelcityForDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.zero();
    shooter.flywheelPercent(0);
  }
}
