package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Limelight;
import frc.robot.RobotContainer;
import frc.robot.commands.shoot.AutoShoot;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class RealAuto extends SequentialCommandGroup {
    public RealAuto(DriveTrain driveTrain, Shooter shooter, Hood hood, BallHandler intake, Turret turret, Limelight limelight, RobotContainer container){
        addCommands(new Autonomous(driveTrain, shooter, hood, intake, turret, limelight, container), new AutoShoot(turret, hood, shooter, intake, limelight, container));
    }
    
}
