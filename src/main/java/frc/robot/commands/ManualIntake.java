// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.Intake.*;

public class ManualIntake extends CommandBase {
  private final Intake intake;
  private final RobotContainer container;
  /** Creates a new ShooterControl. */
  public ManualIntake(Intake intake, RobotContainer container) {
    this.intake = intake;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int pov = container.getOperatorJoystick().getPOV();
    System.out.println(pov);
    if (pov == 0) {
      intake.setPercent(INTAKE_SPEED);
    } else if (pov == 180) {
      intake.setPercent(-INTAKE_SPEED);
    } else {
      intake.setPercent(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
