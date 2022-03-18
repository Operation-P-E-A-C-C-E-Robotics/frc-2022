package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class EmergencyReverse extends CommandBase{

  private BallHandler intake;

  public EmergencyReverse(BallHandler intake){
    this.intake = intake;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setAll(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setAll(0);
  }
}
