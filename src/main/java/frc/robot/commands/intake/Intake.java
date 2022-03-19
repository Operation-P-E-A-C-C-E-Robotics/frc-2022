package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class Intake extends CommandBase{

  private BallHandler intake;

  public Intake(BallHandler intake){
    this.intake = intake;

    addRequirements(intake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake(1);
    intake.setTraversal(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
    intake.setTraversal(0);
  }
}
