package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Flywheel;

public class ReverseTrigger extends CommandBase{
    private BallHandler intake;
    private Flywheel flywheel;

    public ReverseTrigger(Flywheel flywheel, BallHandler intake){
        this.intake = intake;
        this.flywheel = flywheel;

        addRequirements(intake, flywheel);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intake.setTrigger(-1);
      intake.setTraversal(0);
      flywheel.flywheelVelocity(-400);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.setTrigger(0);
      intake.setTraversal(0);
      flywheel.flywheelPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
