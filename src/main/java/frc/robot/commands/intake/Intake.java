package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    intake.armsUp();
    // CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new WaitCommand(2), new InstantCommand(() -> intake.armsUp(), intake)));
  }
}
