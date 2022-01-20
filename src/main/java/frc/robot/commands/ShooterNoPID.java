package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterNoPID extends CommandBase{
    private Shooter shooter;
  /** Creates a new ShooterControl. */
  public ShooterNoPID(Shooter shooter, Joystick joystick) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    SmartDashboard.putNumber("shooter velocity", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Replace with PID Control
    shooter.flywheelPercent(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheelPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    private void getJoystickSpeed(Joystick joystick) {
      double raw = joystick.getThrottle();
      reutrn(raw +1 /2);
    }
}
