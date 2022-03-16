// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.lib.util.TargetTracker;
import frc.robot.subsystems.Hood;

public class AutoHood extends CommandBase {
  private final Hood hood;
  // private final Limelight limelight;
  private TargetTracker target;

  /** Creates a new AutoAim. */
  public AutoHood(Hood hood, TargetTracker target) {
    this.hood = hood;
    // this.limelight = limelight;
    this.target = target;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // limelight.setModeVision();
    //   limelight.setLedOn();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(limelight.hasTarget() == 1){
      hood.setHoodForDistance(target.getTargetDistance());
    // } else{
    //   hood.zero();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // limelight.setModeDrive();
    // limelight.setLedOff();
    hood.zero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
