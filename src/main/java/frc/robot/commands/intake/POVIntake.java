// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Flywheel;

public class POVIntake extends CommandBase {
  private final BallHandler intake;
  private final RobotContainer container;
  private final boolean revshooter;
  private Flywheel flywheel;
  /** Creates a new ShooterControl. */
  public POVIntake(BallHandler intake, Flywheel flywheel, RobotContainer container, boolean revshooter) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.container = container;
    this.revshooter = revshooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int pov = container.getOperatorJoystick().getPOV();
    if (pov == 180) {
      //intake
      intake.armsDown();
      intake.setIntake(1);
      if(revshooter && CommandScheduler.getInstance().requiring(flywheel) == null){
        CommandScheduler.getInstance().schedule(new RampFlywheel(flywheel).withTimeout(10));
      }
    } else if (pov == 135 || pov == 225){
      intake.setTraversal(1);
      intake.setIntake(1);
      if(revshooter && CommandScheduler.getInstance().requiring(flywheel) == null){
        CommandScheduler.getInstance().schedule(new RampFlywheel(flywheel).withTimeout(10));
      }
    } else if (pov == 90 || pov == 270){
      intake.setTraversal(1);
    } else if (pov == 0) {
      //raise arms
      intake.armsUp();
      intake.setAll(0);
    } else {
      intake.setIntake(0);
      intake.setTraversal(0);
      intake.setTrigger(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
