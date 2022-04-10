package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAngle extends CommandBase{
    private Turret turret;
    private Rotation2d angle;

    public TurretAngle(Turret turret, Rotation2d angle){
        this.turret = turret;
        this.angle = angle;

        addRequirements(turret);
    }

    @Override
    public void initialize(){
        turret.setTurretRotations(angle.getDegrees()/360);
    }

    @Override
    public void execute(){
        turret.setTurretRotations(angle.getDegrees()/360);
    }

    @Override
    public boolean isFinished(){
        return turret.ready();
    }
}
