// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oldCommands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class ManualHood extends CommandBase {
  private Hood hood;
  private RobotContainer container;
  private double timer = 0;
  boolean hasControl = false;
  /** Creates a new ManualHoodAdjust. */
  public ManualHood(Hood hood, RobotContainer container) {
    this.hood = hood;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//  || (Timer.getFPGATimestamp() - timer) > 1
    if(Math.abs(container.getOperatorJoystick().getY()) > 0.01){
      // hood.setHoodSpeed(-container.getOperatorJoystick().getY() / 3);
      timer = Timer.getFPGATimestamp();  
      hasControl = true;
    } else if ((Timer.getFPGATimestamp() - timer) > 1){
      hasControl = false;
    }
    if (hasControl) hood.setHoodSpeed(-container.getOperatorJoystick().getY() / 3);
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
