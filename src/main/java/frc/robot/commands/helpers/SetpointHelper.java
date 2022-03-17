package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.lib.math.CubicSplineInterpolate;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.AIM_DATA;

public class SetpointHelper extends CommandBase{
    private Flywheel flywheel;
    private Hood hood;
    private Turret turret;
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

        hoodSetpoint = hoodInterp.cubicSplineInterpolate(distance);
        flywheelSetpoint = flywheelInterp.cubicSplineInterpolate(distance);

        SmartDashboard.putNumber("hood setpoint", hoodSetpoint);
        SmartDashboard.putNumber("flywheel setpoint", flywheelSetpoint);
        SmartDashboard.putNumber("distance", distance);

        hood.setHoodPosition(hoodSetpoint);
        flywheel.flywheelVelocity(flywheelSetpoint);
    }

    @Override
    public void execute(){
        if(joystick.getRawAxis(0) > 0.1) flywheelSetpointOffset += joystick.getRawAxis(0);
        if(joystick.getRawAxis(3) > 0.1) hoodSetpointOffset += joystick.getRawAxis(3);
        distance = camera.getTargetDistance();

        hoodSetpoint = hoodInterp.cubicSplineInterpolate(distance);
        flywheelSetpoint = flywheelInterp.cubicSplineInterpolate(distance);

        hoodSetpoint += hoodSetpointOffset;
        flywheelSetpoint += flywheelSetpointOffset;

        SmartDashboard.putNumber("hood setpoint", hoodSetpoint);
        SmartDashboard.putNumber("flywheel setpoint", flywheelSetpoint);
        SmartDashboard.putNumber("distance", distance);

        hood.setHoodPosition(hoodSetpoint);
        flywheel.flywheelVelocity(flywheelSetpoint);

    }

    @Override
    public void end(boolean i){
        hood.setHoodSpeed(0);
        flywheel.flywheelPercent(0);
    }
}
