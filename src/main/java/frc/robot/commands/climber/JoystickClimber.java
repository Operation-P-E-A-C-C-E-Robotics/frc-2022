package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class JoystickClimber extends CommandBase{
  Climber climber;
  private RobotContainer container;
  private Joystick joystick;

  public JoystickClimber(Climber climber, Joystick joystick, RobotContainer container) {
    this.climber = climber;
    this.container = container;
    this.joystick = joystick;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber.setLiftPercent(joystick.getRawAxis(3));
      // climber.setArmPercent(operatorJoystick.getRawAxis(2));
  }
}
