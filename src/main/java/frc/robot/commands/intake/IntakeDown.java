package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class IntakeDown extends CommandBase{

  private BallHandler intake;

  public IntakeDown(BallHandler intake){
    this.intake = intake;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    intake.armsDown();
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
