package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.DriveTrain;

public class DrivebaseAutoAim extends CommandBase{
    private DriveTrain driveTrain;

    private Limelight limelight;

    
    /** Creates a new ShooterControl. */
    public DrivebaseAutoAim(DriveTrain driveTrain, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      limelight.setModeVision();
        // prev = limelight.getTargetOffsetY();
    }
    
    private final double kP = 0.03;
    private double p = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1){
        double offsetY = limelight.getTargetOffsetY();
        SmartDashboard.putNumber("offset", offsetY);

        p = offsetY * kP;

        double rot = p;

        SmartDashboard.putNumber("rot", rot);

         driveTrain.percentDrive(-rot, rot);
    

        // prev = offsetY;
    } else {
        driveTrain.percentDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setModeDrive();
    driveTrain.percentDrive(0, 0);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
