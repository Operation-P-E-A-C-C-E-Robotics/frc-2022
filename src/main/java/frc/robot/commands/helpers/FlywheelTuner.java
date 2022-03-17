package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import static frc.robot.Constants.Shooter.*;

public class FlywheelTuner extends CommandBase{
    private Flywheel flywheel;
    private double velocity = 0;
    private Joystick joystick;

    public FlywheelTuner(Flywheel flywheel, Joystick testJoystick){
        this.flywheel = flywheel;
        this.joystick = testJoystick;

        addRequirements(flywheel);

        SmartDashboard.putNumber("flywheel kp", FLYWHEEL_kP);
        SmartDashboard.putNumber("flywheel ki", FLYWHEEL_kI);
        SmartDashboard.putNumber("flywheel kd", FLYWHEEL_kD);
        SmartDashboard.putNumber("flywheel kf", FLYWHEEL_kF);
    }

    @Override
    public void execute(){
        flywheel.configTalonGains(
            SmartDashboard.getNumber("flywheel kf", FLYWHEEL_kF),
            SmartDashboard.getNumber("flywheel kp", FLYWHEEL_kP),
            SmartDashboard.getNumber("flywheel ki", FLYWHEEL_kI),
            SmartDashboard.getNumber("flywheel kd", FLYWHEEL_kD)
        );
        velocity += joystick.getRawAxis(3) * 20; //todo get axis
        flywheel.flywheelVelocity(velocity);
        SmartDashboard.putNumber("flyweel velocity", flywheel.getFlywheelVelocity());
        SmartDashboard.putNumber("flywheel error", flywheel.getFlywheelError());
    }
}
