package frc.robot.commands.shooter.setpoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Turret;
import frc.robot.RobotContainer;

public class SetpointBase extends CommandBase{
    private double velocity;
    private double angle;
    private boolean withLimelight;
    private double curretTurretPosition, 
    targetTurretPosition = Double.NaN;

    private Flywheel shooter;
    private Hood hood;
    private Limelight limelight;
    private Turret turret;
    private RobotContainer container;

    public SetpointBase(Flywheel shooter, Hood hood, Turret turret, Limelight camera, RobotContainer container, double velocity, double angle, boolean withLimelight){
        this.shooter = shooter;
        this.hood = hood;
        this.limelight = camera;
        this.turret = turret;
        this.velocity = velocity;
        this.angle = angle;
        this.withLimelight = withLimelight;
        this.container = container;
        addRequirements(shooter, hood);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (withLimelight == true) {
    limelight.setModeVision();
    limelight.setLedOn();
    }
    targetTurretPosition = Double.NaN;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.flywheelVelocity(velocity);
    hood.setHoodPosition(angle);

    if(limelight.hasTarget() == 1 && withLimelight){
      curretTurretPosition = turret.getPosition();
      double deltaX = limelight.getTargetOffsetX() / 360;
      double newTargetTurretPosition = curretTurretPosition + deltaX; //todo figure out what's flipped
       

      if(Double.isNaN(targetTurretPosition) || Math.abs(deltaX) > 5){
        targetTurretPosition = newTargetTurretPosition;
      } else {
        targetTurretPosition += (newTargetTurretPosition - targetTurretPosition) / 10;
      }

      turret.setTurretRotations(targetTurretPosition);
    } else {
    turret.setTurretPercent(container.getOperatorJoystick().getX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheelPercent(0);
    limelight.setModeDrive();
    limelight.setLedOff();
    turret.setTurretPercent(0.0);
    hood.zero();
  }
}
