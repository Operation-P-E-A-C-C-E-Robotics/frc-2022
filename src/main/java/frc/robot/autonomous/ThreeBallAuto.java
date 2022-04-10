package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class ThreeBallAuto extends SequentialCommandGroup{
    private Flywheel flywheel;

    public ThreeBallAuto(DriveTrain driveTrain, BallHandler intake, Flywheel flywheel, Turret turret, Hood hood, Limelight limelight, RobotContainer container, Pigeon pigeon){
        this.flywheel = flywheel;
        flywheel.setDefaultCommand(new RampFlywheel(flywheel));
        addCommands(
            new DriveDistance(driveTrain, container, 0.83, 0.25)
                .raceWith(new Intake(intake)),
            new Intake(intake).withTimeout(0.5),
            new TurnAngle(driveTrain, pigeon, container, Rotation2d.fromDegrees(81), 0.3),
            new TurretAngle(turret, Rotation2d.fromDegrees(-100)),
            new AutoShoot(flywheel, hood, turret, intake, limelight, container).withTimeout(4),
            new DriveDistance(driveTrain, container, 2, 0.5)
                .raceWith(new Intake(intake)),
            new Intake(intake).withTimeout(0.8),
            new TurretAngle(turret, Rotation2d.fromDegrees(60)),
            new AutoShoot(flywheel, hood, turret, intake, limelight, container)
        );
    }

    @Override
    public void end(boolean e){
        flywheel.setDefaultCommand(null);
    }
}
