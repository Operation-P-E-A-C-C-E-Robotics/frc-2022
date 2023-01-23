// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallOffLine extends SequentialCommandGroup {
  /** Creates a new OneBallOffLine. */
  public OneBallOffLine(DriveTrain driveTrain, Flywheel shooter, Hood hood, Turret turret, BallHandler intake, Limelight limelight, Pigeon pigeon, RobotContainer container) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoShoot(shooter, hood, turret, intake, driveTrain, limelight, container).withTimeout(8), new DriveDistance(driveTrain, pigeon, container, 1, 0.25));
  }
}
