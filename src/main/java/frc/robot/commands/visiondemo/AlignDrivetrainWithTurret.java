    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visiondemo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.DriveTrain.Gear;
import edu.wpi.first.wpilibj.Joystick;

public class AlignDrivetrainWithTurret extends CommandBase {
 private final DriveTrain driveTrain;
  private final RobotContainer container;

  private final double p = 10;
  private final double i = 0.0001;
  private final double distP = 0.1;
  private final double distI = 0.01;
  private final double targetArea = 20;
private Turret turret;
private Limelight limelight;

  /** Creates a new drive. */
  public AlignDrivetrainWithTurret(DriveTrain driveTrain, Turret turret, Limelight limelight, RobotContainer container) {
    this.driveTrain = driveTrain;
    this.container = container;
    this.turret = turret;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }
  

  @Override
  public void end(boolean i){
    //   driveTrain.percentDrive(0, 0);
  }

  double integralArea = 0;
  double integralDistance = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      integralArea = Util.limit(integralArea, 0.5);
      integralDistance = Util.limit(integralDistance, 0.5);

      double error = turret.getPosition();
      integralArea += error > 0 ? i : -i;
      double rot = (error * p) + integralArea;
      rot = Util.limit(rot, 0.7);
    
      double distError = limelight.getTargetLongSidelength() - targetArea;
      integralDistance += distError > 0 ? distI : -distI;
      if(distError < 0.1) integralDistance = 0;
      double spd = Util.limit((distError * distP) + integralDistance, 0.7);
      driveTrain.arcadeDrive(spd, rot);

      if(limelight.hasTarget() == 0){
        driveTrain.percentDrive(0, 0);
        integralArea = 0;
        integralDistance = 0;
      }
  }
}
