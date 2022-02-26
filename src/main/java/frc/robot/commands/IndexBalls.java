// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import static frc.robot.Constants.Intake.*;

public class IndexBalls extends CommandBase {
  private final BallHandler ballHandler;
  private final RobotContainer container;

  boolean allianceIsRed = false; //TODO change to reflect FMS data

  boolean ballInTraversal = false, 
          ballInTrigger = false,
          ballInTraversalCorrectColor = true,
          ballInTriggerCorrectColor = true;

  State state = State.NOT_RUNNING;

  double counts_at_traversal = 0;
  /** Creates a new ShooterControl. */
  public IndexBalls(BallHandler intake, RobotContainer container) {
    this.ballHandler = intake;
    this.container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballInTraversal = ballHandler.ballInTraversal();
    ballInTrigger = ballHandler.ballInTrigger();
    ballInTraversalCorrectColor = ballHandler.getTraversalBallColor() == allianceIsRed;
    ballInTriggerCorrectColor = ballHandler.getTriggerBallColor() == allianceIsRed;

    if(state == State.FLOOR_TO_TRAVERSAL){
      ballHandler.setIntake(1);
      ballHandler.setTraversal(0.25);
    }

    if(state == State.TRAVERSAL_TO_TRIGGER){
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum State{
    FLOOR_TO_TRAVERSAL,
    TRAVERSAL_TO_FLOOR,
    TRAVERSAL_TO_TRIGGER,
    CENTER_IN_TRIGGER,
    FULL,
    NOT_RUNNING
  }
}
