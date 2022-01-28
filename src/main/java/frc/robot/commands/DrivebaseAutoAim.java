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
    @Override
    public void initialize() {
      limelight.setModeVision();
      limelight.setLedOn();
        prev = limelight.getTargetOffsetY();
    }

    private boolean isFirst = true;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1){
        double offsetY = limelight.getTargetOffsetX();
        

        // private final double kI = 0;
        double kP = 0.02;
        double p = offsetY * kP;

        // private double i = 0;
        double kD = 0.00000001;
        double d = -(offsetY - prev) * kD;

        if(isFirst) d = 0;


        double rot = p + d;

         driveTrain.percentDrive(rot, -rot);
    

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
