// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.Intake.*;

public class ManualTrigger extends CommandBase {
  private final BallHandler intake;
  private Turret turret;
  private Limelight limelight;

  private double curretTurretPosition, 
  targetTurretPosition = Double.NaN;
  /** Creates a new ShooterControl. */
  public ManualTrigger(BallHandler intake, Turret turret, Limelight limelight) {
    this.intake = intake;
    this.turret = turret;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      intake.setTraversal(0.35);
      intake.setTrigger(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
    intake.setTraversal(0);
    intake.setTrigger(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
