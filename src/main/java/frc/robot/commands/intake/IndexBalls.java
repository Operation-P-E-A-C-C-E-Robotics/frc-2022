// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import static frc.robot.Constants.Intake.*;

public class IndexBalls extends CommandBase {
  private final BallHandler ballHandler;
  private final RobotContainer container;

  boolean allianceIsRed = false; //TODO change to reflect FMS data

  boolean ejectingball = false,
          ballInTraversal = false,
          ballInTraversalCorrectColor = false;
  
  double ejectingTime = 0;

  // State state = State.NOT_RUNNING;
  boolean ejectingBall = false;

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
    ballInTraversalCorrectColor = ballHandler.getTraversalBallColor() == allianceIsRed;
    
    if(ejectingBall) ejectingBall = (Timer.getFPGATimestamp() - ejectingTime) > 3;
    
    if(ejectingBall){
      ballHandler.armsUp();
      ballHandler.setTraversal(-1);
      ballHandler.setIntake(0);
    } else {
      ballHandler.armsDown();

      if(ballInTraversal && !ballInTraversalCorrectColor) {
        ejectingBall = true;
        ejectingTime = Timer.getFPGATimestamp();
      }

      ballHandler.setTraversal(0.2);
      ballHandler.setIntake(0.5);
    }

    // switch (state){
    //   case FLOOR_TO_TRAVERSAL:
    //     if(ballInTraversal){
    //       if(ballInTraversalCorrectColor){
    //         if(!ballInTrigger) state = State.TRAVERSAL_TO_TRIGGER;
    //         else state = State.FULL;
    //       } else state = State.TRAVERSAL_TO_FLOOR;
    //     }
    //   case CENTER_IN_TRIGGER:
    //     if(ballInTriggerCentered) state = State.NOT_RUNNING;
    //     break;
    //   case FULL:
    //     if (!ballInTrigger) state = State.TRAVERSAL_TO_TRIGGER;
    //     if (!ballInTraversal) state = State.FLOOR_TO_TRAVERSAL;
    //     break;
    //   case NOT_RUNNING:
    //     if (!ballInTrigger) state = State.TRAVERSAL_TO_TRIGGER;
    //     if (!ballInTraversal) state = State.FLOOR_TO_TRAVERSAL;
    //     break;
    //   case TRAVERSAL_TO_FLOOR:
    //     if (ballReachedFloor) state = State.NOT_RUNNING;
    //     break;
    //   case TRAVERSAL_TO_TRIGGER:
    //     if (ballInTrigger) 
    //     break;
    //   default:
    //     break;
    // }


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
