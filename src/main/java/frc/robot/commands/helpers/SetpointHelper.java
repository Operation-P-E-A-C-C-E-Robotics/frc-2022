package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicSplineInterpolate;
import frc.lib.sensors.Limelight;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import static frc.robot.Constants.AIM_DATA;

public class SetpointHelper extends CommandBase{
    private Flywheel flywheel;
    private Hood hood;
    private Limelight camera;
    private Joystick joystick;

    private CubicSplineInterpolate hoodInterp;
    private CubicSplineInterpolate flywheelInterp;

    private double flywheelSetpoint = 0,
                    hoodSetpoint = 0;

    private double hoodSetpointOffset = 0,
                    flywheelSetpointOffset = 0;

    private double distance;

    public SetpointHelper(Flywheel flywheel, Hood hood, Limelight camera, Joystick testingJoystick){
        this.flywheel = flywheel;
        this.hood = hood;
        this.camera = camera;
        this.joystick = testingJoystick;

        hoodInterp = new CubicSplineInterpolate();
        hoodInterp.setSamples(AIM_DATA[0], AIM_DATA[2]);

        flywheelInterp = new CubicSplineInterpolate();
        flywheelInterp.setSamples(AIM_DATA[0], AIM_DATA[1]);

        addRequirements(flywheel, hood);
    }

    @Override
    public void initialize(){
        camera.setLedOn();
        camera.setModeVision();
        distance = camera.getTargetDistance();

        // hoodSetpoint = hoodInterp.cubicSplineInterpolate(distance);
        // flywheelSetpoint = flywheelInterp.cubicSplineInterpolate(distance);

        SmartDashboard.putNumber("hood setpoint", 0);
        SmartDashboard.putNumber("flywheel setpoint", 0);
        SmartDashboard.putNumber("distance", distance);

        hood.setHoodPosition(hoodSetpoint);
        flywheel.flywheelVelocity(flywheelSetpoint);
    }

    @Override
    public void execute(){
        flywheelSetpointOffset += joystick.getRawAxis(1);
        hoodSetpointOffset += joystick.getRawAxis(3);
        distance = camera.getTargetDistance();

        // hoodSetpoint = hoodInterp.cubicSplineInterpolate(distance);
        // flywheelSetpoint = flywheelInterp.cubicSplineInterpolate(distance);

        // hoodSetpoint += hoodSetpointOffset;
        // flywheelSetpoint += flywheelSetpointOffset;

        // SmartDashboard.putNumber("interp hood setpoint", hoodSetpoint);
        // SmartDashboard.putNumber("interp flywheel setpoint", flywheelSetpoint);
        hood.setHoodPosition(SmartDashboard.getNumber("hood setpoint", 0));
        flywheel.flywheelVelocity(SmartDashboard.getNumber("flywheel setpoint", 0));
        
        SmartDashboard.putNumber("distance", distance);
    }
    
    @Override
    public void end(boolean i){
        SmartDashboard.putNumber("hood setpoint", hoodSetpointOffset);
        SmartDashboard.putNumber("flywheel setpoint", flywheelSetpointOffset);
        hood.setHoodSpeed(0);
        flywheel.flywheelPercent(0);
    }
}
