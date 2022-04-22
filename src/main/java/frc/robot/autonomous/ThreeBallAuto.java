package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.IntakeDown;
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
        // flywheel.setDefaultCommand(new RampFlywheel(flywheel));

        //path
        // Command driveOffLine = new DriveDistance(driveTrain, container, 0.83, 0.25),
        //         turretForFirstBall = new TurretAngle(turret, Rotation2d.fromDegrees(-100)),
        //         turnToSecondBall = new TurnAngle(driveTrain, pigeon, container, Rotation2d.fromDegrees(81), 0.3),
        //         driveToSecondBall = new DriveDistance(driveTrain, container, 2, 0.5),
        //         turretForSecondBall = new TurretAngle(turret, Rotation2d.fromDegrees(60));

        // //actions
        // Command runIntake = new Intake(intake),
        //         shoot = new AutoShoot(flywheel, hood, turret, intake, limelight, container),
        //         ramp = new RampFlywheel(flywheel);

        // addCommands(driveOffLine.raceWith(runIntake, ramp),
        //             runIntake.raceWith(ramp.withTimeout(0.5)),
        //             turretForFirstBall.raceWith(ramp),
        //             turnToSecondBall.raceWith(ramp),
        //             shoot,
        //             driveToSecondBall.raceWith(runIntake, ramp),
        //             runIntake.withTimeout(0.8).alongWith(turretForSecondBall).raceWith(ramp),
        //             shoot);
        addCommands(
            new DriveDistance(driveTrain, pigeon, container, 0.83, 0.25)
                .alongWith(new TurretAngle(turret, Rotation2d.fromDegrees(-100)))
                .raceWith(
                    new Intake(intake), 
                    new RampFlywheel(flywheel)
                ),
            new Intake(intake)
                .withTimeout(0.5)
                .raceWith(new RampFlywheel(flywheel)),
            new TurnAngle(driveTrain, pigeon, container, Rotation2d.fromDegrees(93.5), 0.2)
                .raceWith(new RampFlywheel(flywheel)),
            new AutoShoot(flywheel, hood, turret, intake, driveTrain, limelight, container).withTimeout(2),
            new DriveDistance(driveTrain, pigeon, container, 2, 0.5)
                .alongWith(new TurretAngle(turret, Rotation2d.fromDegrees(-60)))
                .raceWith(
                    new Intake(intake),
                    new RampFlywheel(flywheel)
                ),
            new Intake(intake)
                .withTimeout(0.8)
                .raceWith(new RampFlywheel(flywheel)),
            // new IntakeDown(intake),
            new AutoShoot(flywheel, hood, turret, intake, driveTrain,limelight, container)
                .withTimeout(2)
        );
    }

    @Override
    public void end(boolean e){
        // flywheel.setDefaultCommand(null);
    }
}
