package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.subsystems.DriveTrain;

public class DrivebaseAutoAim extends CommandBase{
    private final DriveTrain driveTrain;

    private final Limelight limelight;

    
    /** Creates a new ShooterControl. */
    public DrivebaseAutoAim(DriveTrain driveTrain, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }
    
    // Called when the command is initially scheduled.
    private double prev = 0;
    private double avg = 0;
    @Override
    public void initialize() {
      limelight.setModeVision();
      limelight.setLedOn();
        prev = limelight.getTargetOffsetY();
        avg = limelight.getTargetOffsetY();
    }
    
    private final double kP = 0.015;
    private final double kD = 0;//.00000001;
    private final double kI = 0;
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private boolean isFirst = true;
    private double ramp = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1){
        double offsetY = limelight.getTargetOffsetX();
        avg += offsetY;
        avg /= 2;

        p = avg * kP;

        d = -(avg - prev) * kD;

        if(isFirst) d = 0;


        double rot = p + d;

         driveTrain.percentDrive(-rot, rot);
    

        // prev = offsetY;
        isFirst = false;
    } else {
        isFirst = true;
        driveTrain.percentDrive(-0.1, 0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setModeDrive();
    limelight.setLedOff();
    driveTrain.percentDrive(0, 0);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
