// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;

public class TriggerWhenReady extends CommandBase {
  private final Turret turret;
  private final Hood hood;
  private final Flywheel shooter;
  private final Limelight limelight;
  private BallHandler intake;

  /** Creates a new AutoAim. */
  public TriggerWhenReady(Turret turret, Hood hood, Flywheel shooter, BallHandler intake, Limelight limelight) {
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.limelight = limelight;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1){
      if(turret.ready() && shooter.ready() && hood.ready()){
        intake.setTrigger(1);
        intake.setTraversal(0.5);
      } else{
        intake.setTrigger(0);
        intake.setTraversal(0);
      }
    } else{
        intake.setTrigger(0);
        intake.setTraversal(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
