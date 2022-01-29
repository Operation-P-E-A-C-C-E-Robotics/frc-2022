// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Turret.*;

public class Turret extends SubsystemBase {
    private final CANSparkMax turretMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;
    private final DigitalInput zeroSwitch;

    private double setpoint = 0;
    private final double TURRET_RATIO = 0; //todo get ratio


    /** Creates a new Shooter. */
    public Turret() {
        turretMotor = new CANSparkMax(TURRET_CONTROLLER_PORT, MotorType.kBrushless);
        encoder = turretMotor.getEncoder();
        pidController = turretMotor.getPIDController();
        turretMotor.setInverted(false); //TODO i want positive numbers to make clockwise rotation
        // turretMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
        // turretMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

        new GainSetter(pidController)
        .ff(kFF)
        .p(kP)
        .i(kI)
        .d(kD)
        .iz(kIz)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        zeroSwitch = new DigitalInput(0); //todo set up
    }

    public GainSetter getGainSetter(){
        return new GainSetter(pidController);
    }

    public void turretRotations(double rotations){
        rotations *= TURRET_RATIO; //convert from motor rotations to turret rotations
        pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        setpoint = rotations;
    }

    //DONT KNOW IF THIS WILL WORKEY
    public void turretVelocity(double velocity){
        pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * set the turret motor power
     * @param speed the speed (-1 to 1)
     */
    public void turretPercent(Double speed) {
        turretMotor.set(speed);
    }

    public double getPosition(){
        return encoder.getPosition() / TURRET_RATIO; //should it be / or *??
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret current", turretMotor.getOutputCurrent());
        SmartDashboard.putNumber("turret temperature", turretMotor.getMotorTemperature());
        SmartDashboard.putNumber("turret position", encoder.getPosition());
        SmartDashboard.putNumber("turret position error", setpoint - encoder.getPosition());

        if(zeroSwitch.get()){
            encoder.setPosition(0);
        }
    }

    public static class GainSetter {
        public SparkMaxPIDController controller;

        public GainSetter(SparkMaxPIDController controller) {
            this.controller = controller;
        }

        public GainSetter ff(double kFF) {
            controller.setFF(kFF);
            return this;
        }
        public GainSetter p(double kP) {
            controller.setP(kP);
            return this;
        }
        public GainSetter i(double kI) {
            controller.setI(kI);
            return this;
        }

        public GainSetter d(double kD) {
            controller.setD(kD);
            return this;
        }

        public GainSetter iz(double kIz){
            controller.setIZone(kIz);
            return this;
        }

        public GainSetter outputRange(double min, double max) {
            controller.setOutputRange(min, max);
            return this;
        }

    }
}