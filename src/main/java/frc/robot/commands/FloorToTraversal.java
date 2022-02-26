// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import static frc.robot.Constants.Intake.*;

public class FloorToTraversal extends CommandBase {
  private final BallHandler ballHandler;
  private final RobotContainer container;

  boolean allianceIsRed = false; //TODO change to reflect FMS data

  boolean ballInTraversal = false, 
          ballInTrigger = false,
          ballInTraversalCorrectColor = true,
          ballInTriggerCorrectColor = true;

    boolean centering = false;

  double counts_at_center = 0;
  /** Creates a new ShooterControl. */
  public FloorToTraversal(BallHandler intake, RobotContainer container) {
    this.ballHandler = intake;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      ballHandler.setIntake(1);
      ballHandler.setTraversal(0.25);
      centering = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballInTraversal = ballHandler.ballInTraversal();
    ballInTrigger = ballHandler.ballInTrigger();
    ballInTraversalCorrectColor = ballHandler.getTraversalBallColor() == allianceIsRed;
    ballInTriggerCorrectColor = ballHandler.getTriggerBallColor() == allianceIsRed;

    if(ballInTraversal){
        centering = true;
        counts_at_center = ballHandler.getTraversalPosition();
        ballHandler.setIntakePosition(counts_at_center);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return centering && Math.abs(Math.abs(ballHandler.getTraversalPosition()) - Math.abs(counts_at_center)) < 10;
  }
}
