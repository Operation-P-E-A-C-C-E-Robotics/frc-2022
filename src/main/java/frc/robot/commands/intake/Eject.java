package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class Eject extends CommandBase{

  private BallHandler intake;

  public Eject(BallHandler intake){
    this.intake = intake;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.armsUp();
    intake.setTraversal(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setTraversal(0);
  }
}
