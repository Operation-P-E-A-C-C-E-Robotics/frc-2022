// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Traversal;
import frc.robot.subsystems.BallHandler;
import static frc.robot.Constants.Intake.*;

public class ManualIntake extends CommandBase {
  private final BallHandler intake;
  private final RobotContainer container;
  /** Creates a new ShooterControl. */
  public ManualIntake(BallHandler intake, RobotContainer container) {
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
    //System.out.println(pov);
    if (pov == 0) {
      intake.setIntake(INTAKE_SPEED);
    } else if (pov == 180) {
      intake.setIntake(-INTAKE_SPEED);
    } else if (container.getDriverJoystick().getRawButton(1) == true) {
      intake.setIntake(INTAKE_SPEED);
    }else {
      intake.setIntake(0);
    }
    


    if (container.getOperatorJoystick().getRawButton(5) == true) {
      intake.setTraversal(-0.5);
    } else if (container.getDriverJoystick().getRawButton(2) == true) {
      intake.setTraversal(-0.5);
    } else {
      intake.setTraversal(0);
    }


    if (container.getOperatorJoystick().getRawButton(7) == true) {
      intake.setTrigger(1);
    } else {
      intake.setTrigger(0);
    }


   if (container.getOperatorJoystick().getRawButton(10) == true) {
      intake.armsUp();
    }

    if (container.getOperatorJoystick().getRawButton(9) == true) {
      intake.armsDown();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
