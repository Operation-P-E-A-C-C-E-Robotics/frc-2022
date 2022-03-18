package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.sensors.Limelight;
import frc.lib.util.TargetTracker;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class AutoAim extends ParallelCommandGroup{
    public AutoAim(Flywheel flywheel, Hood hood, Turret turret, Limelight limelight, TargetTracker target){
        addCommands(new AutoFlywheel(flywheel, limelight), 
                    new AutoHood(hood, target),
                    new AutoTurret(turret, target));
    }
}

