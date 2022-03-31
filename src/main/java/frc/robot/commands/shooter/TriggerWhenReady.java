// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OldTurret;

public class TriggerWhenReady extends CommandBase {
  private final OldTurret turret;
  private final Hood hood;
  private final Flywheel shooter;
  private final Limelight limelight;
  private BallHandler intake;
  private double timer = 0;

  /** Run the trigger once everything is in position to shoot */
  public TriggerWhenReady(OldTurret turret, Hood hood, Flywheel shooter, BallHandler intake, Limelight limelight) {
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.limelight = limelight;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget() == 1 && turret.ready() && shooter.ready() && hood.ready()){
      twosecondtriggerwheel();
      //   intake.setTrigger(1);
    //   intake.setTraversal(0.6);
    // } else{
    //   intake.setTrigger(0);
    //   intake.setTraversal(0.15);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setTrigger(0);
    intake.setTraversal(0);
  }

  private void twosecondtriggerwheel() {
    timer = Timer.getFPGATimestamp();

    intake.setTrigger(1);
    intake.setTraversal(0.6);
    Timer.delay(0.5);
    intake.setTrigger(0);
    intake.setTraversal(0.15);

  }

}
