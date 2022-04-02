package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class JoystickClimber extends CommandBase{
  Climber climber;
  private Joystick joystick;
  private Joystick driverJoystick;

  public JoystickClimber(Climber climber, Joystick joystick, Joystick driverJoystick, RobotContainer container) {
    this.climber = climber;
    this.joystick = joystick;
    this.driverJoystick = driverJoystick;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber.setLiftPercent(joystick.getY());
      double driverPOV = driverJoystick.getPOV();
      // if(driverPOV == 180) {
      //   climber.setLiftPercent(0.5);
      //   climber.armsOut();
      // }
      // if(driverPOV == 0) {
      //   climber.setLiftPercent(-0.5);
      //   climber.armsIn();
      // }

      if((-joystick.getY()) > 0.01){
        climber.armsIn();
      } else {
        climber.armsOut();
      }

      // SmartDashboard.putNumber("lift position", climber.getLiftPosition());
      // if(climber.getLiftPosition() > -135000){
      //   climber.armsOut();
      // } else {
      //   climber.armsIn();
      // }

      // if(!joystick.getRawButton(7) && !driverJoystick.getRawButton(8)){
      //   climber.armsOut();
      // }
      // else{
      //   climber.armsIn();
      // }
      // climber.setArmPercent(operatorJoystick.getRawAxis(2));
  }
}
