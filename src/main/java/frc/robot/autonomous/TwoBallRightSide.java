// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeNoTraversal;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallRightSide extends SequentialCommandGroup {
  /** Creates a new TwoBallRightSide. */
  public TwoBallRightSide(DriveTrain driveTrain, Pigeon pigeon, Flywheel shooter, Hood hood, Turret turret, BallHandler intake, Limelight limelight, RobotContainer container) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveDistance(driveTrain, pigeon, container, 3.5, 0.25).withTimeout(4).raceWith(new Intake(intake), new RampFlywheel(shooter)), new AutoShoot(shooter, hood, turret, intake, driveTrain, limelight, container).alongWith(new IntakeUp(intake)));
  }
}

