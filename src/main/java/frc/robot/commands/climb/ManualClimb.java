package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ManualClimb extends CommandBase{
    Climber climber;
    private RobotContainer container;

  public ManualClimb(Climber climber, RobotContainer container) {
    this.climber = climber;
    this.container = container;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Joystick operatorJoystick = container.getOperatorJoystick();
      climber.setLiftPercent(operatorJoystick.getRawAxis(3));
      climber.setArmPercent(operatorJoystick.getRawAxis(2));
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
}
