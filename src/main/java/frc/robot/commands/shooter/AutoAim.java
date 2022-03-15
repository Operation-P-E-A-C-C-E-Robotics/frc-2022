package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.Limelight;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class AutoAim extends ParallelCommandGroup{
    public AutoAim(Flywheel flywheel, Hood hood, Turret turret, Limelight limelight){
        addCommands(new AutoFlywheel(flywheel, limelight), 
                    new AutoHood(hood, limelight),
                    new AutoTurret(turret, limelight));
    }
}
