package frc.lib.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
public class Auto extends CommandBase{
    private TimelineAuto auto = new TimelineAuto();

  public Auto(Subsystem... requirements) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      auto.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      auto.update();
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

  public Auto add(Action action){
      double start = auto.getMaxTime();
      double stop = start + action.duration();
      auto.add(new TimedRunner(action, start, stop));
      return this;
  }

  public Auto run(Action action, double start){
    double stop = start + action.duration();
    auto.add(new TimedRunner(action, start, stop));
    return this;
   }

   public Auto run(Action action, double start, double stop) {
     auto.add(new TimedRunner(action, start, stop));
     return this;
   }

  public Auto duration(Action action, double duration){
    double start = auto.getMaxTime();
    double stop = start + duration;
    auto.add(new TimedRunner(action, start, stop));
    return this;
  }

}
