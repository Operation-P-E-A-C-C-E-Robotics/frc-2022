package frc.robot.oldCommands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Flywheel;

public class ShooterReverse extends CommandBase{
    private Flywheel shooter;
    private BallHandler handler;

    public ShooterReverse(Flywheel shooter, BallHandler handler){
        this.shooter = shooter;
        this.handler = handler;

        addRequirements(shooter, handler);
    }

    public void initialize(){
        shooter.flywheelPercent(-0.5);
        handler.setTrigger(-1);
    }

    public void end(){
        handler.setAll(0);
        shooter.flywheelPercent(0);
    }

    public boolean isFinished(){
        return false;
    }
}
