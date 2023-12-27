package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.lib.sensors.Limelight;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import static frc.robot.Constants.AIM_DATA;

public class NewSetpointHelper extends CommandBase{
    private Flywheel flywheel;
    private Hood hood;
    private Limelight camera;
    private Joystick joystick;

    private double distance, hoodSetpoint = 0, flywheelSetpoint = 0;

    public NewSetpointHelper(Flywheel flywheel, Hood hood, Limelight camera, Joystick testingJoystick){
        this.flywheel = flywheel;
        this.hood = hood;
        this.camera = camera;
        this.joystick = testingJoystick;
    }

    @Override
    public void initialize(){
        camera.setLedOn();
        camera.setModeVision();
        distance = camera.getTargetDistance();
        SmartDashboard.putNumber("flywheel setpoint", 0);
        SmartDashboard.putNumber("hood setpoint", 0);
    }

    @Override
    public void execute(){
        distance = camera.getTargetDistance();
        flywheelSetpoint = SmartDashboard.getNumber("flywheel setpoint", 0);
        hoodSetpoint = SmartDashboard.getNumber("hood setpoint", 0);
        SmartDashboard.putNumber("target distance", distance);
        if(joystick.getRawButton(6)){
            flywheel.flywheelVelocity(flywheelSetpoint);
        } else {
            flywheel.flywheelPercent(0);
        }
        hood.setHoodPosition(hoodSetpoint);
    }
    
    @Override
    public void end(boolean i){
        SmartDashboard.putString("setpoint info:", "d: " + distance + 
                                                " \nfl: " + flywheelSetpoint + 
                                                " \nhd: " + hoodSetpoint);
        flywheel.flywheelPercent(0);
    }
}
