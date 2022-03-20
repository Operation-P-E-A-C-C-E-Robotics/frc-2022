package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class IntakeUp extends CommandBase{

    private BallHandler intake;

    public IntakeUp(BallHandler intake){
        this.intake = intake;
    }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intake.armsUp();
  }
}
