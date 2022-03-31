package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import static frc.robot.Constants.HoodConstants.*;

public class HoodTuner extends CommandBase{
    private Hood hood;
    private Joystick joystick;

    private double setpoint = 0;

    private boolean joystickControl;

    public HoodTuner (Hood hood, Joystick testJoystick){
        this.hood = hood;
        this.joystick = testJoystick;

        addRequirements(hood);

        SmartDashboard.putNumber("hood kf", kF);
        SmartDashboard.putNumber("hood kp", kP);
        SmartDashboard.putNumber("hood ki", kI);
        SmartDashboard.putNumber("hood kd", kD);

        SmartDashboard.putNumber("hood setpoint", setpoint);
    }

    @Override
    public void execute(){
        hood.configTalonGains(
            SmartDashboard.getNumber("hood kf", kF), 
            SmartDashboard.getNumber("hood kp", kP), 
            SmartDashboard.getNumber("hood ki", kI), 
            SmartDashboard.getNumber("hood kd", kD)
        );
        // if(joystick.getRawAxis(4) > 0.1) joystickControl = true;
        // if(SmartDashboard.getNumber("hood setpoint", setpoint) != setpoint) joystickControl = false;
        
        if(joystickControl) {
            setpoint += joystick.getRawAxis(4); //TODO get correct axis
            SmartDashboard.putNumber("hood setpoint", setpoint);
        } else {
            setpoint = SmartDashboard.getNumber("hood setpoint", setpoint);
        }
        
        hood.setHoodPosition(setpoint);
        SmartDashboard.putNumber("hood position", hood.getHoodPosition());
        
    }

    @Override
    public void end(boolean i){
        hood.zero();
    }
}
