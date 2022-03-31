package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.sensors.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.OldTurret;

public class AutoShoot extends ParallelCommandGroup{
    public AutoShoot(Flywheel flywheel, Hood hood, OldTurret turret, BallHandler intake, Limelight limelight, RobotContainer container) {
        addCommands(new AutoAim(flywheel, hood, turret, limelight, container.getOdometry().getTarget()),
                    new TriggerWhenReady(turret, hood, flywheel, intake, limelight));
    }
}
