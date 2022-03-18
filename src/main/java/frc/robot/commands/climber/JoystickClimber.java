package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class JoystickClimber extends CommandBase{
  Climber climber;
  private RobotContainer container;

  public JoystickClimber(Climber climber, RobotContainer container) {
    this.climber = climber;
    this.container = container;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Joystick operatorJoystick = container.getOperatorJoystick();
      climber.setLiftPercent(operatorJoystick.getRawAxis(3));
      climber.setArmPercent(operatorJoystick.getRawAxis(2));
  }
}
