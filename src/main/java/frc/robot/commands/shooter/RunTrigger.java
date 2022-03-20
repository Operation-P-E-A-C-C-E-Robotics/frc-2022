package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class RunTrigger extends CommandBase{
  private BallHandler intake;

  public RunTrigger(BallHandler intake){
    this.intake = intake;

    addRequirements(intake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setTrigger(1);
    intake.setTraversal(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.setTrigger(0);
      intake.setTraversal(0);
  }
}
