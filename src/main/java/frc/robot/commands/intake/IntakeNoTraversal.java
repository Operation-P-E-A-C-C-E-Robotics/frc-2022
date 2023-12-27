package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class IntakeNoTraversal extends CommandBase{

  private BallHandler intake;

  public IntakeNoTraversal(BallHandler intake){
    this.intake = intake;

    addRequirements(intake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
    // CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new WaitCommand(2), new InstantCommand(() -> intake.armsUp(), intake)));
  }
}
