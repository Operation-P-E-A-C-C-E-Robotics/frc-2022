package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.Limelight;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Flywheel;

public class FarHumanPlayerBallAuto extends SequentialCommandGroup {
    public FarHumanPlayerBallAuto(DriveTrain driveTrain, Flywheel shooter, Hood hood, BallHandler intake, Turret turret, Limelight limelight, RobotContainer container){
        addCommands(new FarRightBall(driveTrain, shooter, hood, intake, turret, limelight, container), new AutoShoot(shooter, hood, turret, intake, limelight, container));
    }
    
}
