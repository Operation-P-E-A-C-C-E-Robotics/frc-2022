// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Turret.*;

public class Turret extends SubsystemBase {
    private final CANSparkMax turretMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;
    // private final DigitalInput zeroSwitch;

    private double setpoint = 0;
    // private final double TURRET_RATIO = (1/20) * (1/11); //todo get ratio
    private final double MOTOR_ROTS_PER_TURRET_ROT = 242;

    /** Creates a new Shooter. */
    public Turret() {
        turretMotor = new CANSparkMax(TURRET_CONTROLLER_PORT, MotorType.kBrushless);
        encoder = turretMotor.getEncoder();
        pidController = turretMotor.getPIDController();
        turretMotor.setInverted(false);
        // turretMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(0.25 * MOTOR_ROTS_PER_TURRET_ROT));
        // turretMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        turretMotor.setSmartCurrentLimit(10);
        // pidController.setSmartMotionAllowedClosedLoopError(40.0, 0);
        new GainSetter(pidController)
        .ff(kFF)
        .p(kP)
        .i(kI)
        .d(kD)
        .iz(kIz)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        // zeroSwitch = new DigitalInput(0); //todo set up
    }

    public GainSetter getGainSetter(){
        return new GainSetter(pidController);
    }

    public void turretRotations(double rotations){
        // if(encoder.getPosition() - rotations < 40) rotations = encoder.getPosition();
        // rotations = rotations < 0.5 ? rotations : 0.5;
        // rotations = rotations > -0.5 ? rotations : -0.5;
        rotations = ((rotations + 0.5) % 1) - 0.5;
        rotations *= MOTOR_ROTS_PER_TURRET_ROT; //convert from motor rotations to turret rotations
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
        return encoder.getPosition() / MOTOR_ROTS_PER_TURRET_ROT; //should it be / or *??
    }

    public void zero(){
        encoder.setPosition(0);
    }

    public boolean ready(){
        return (Math.abs(setpoint) - Math.abs(encoder.getPosition())) < 3;
    }

    @Override
    public void periodic() {
        // if(zeroSwitch.get()){
        //     encoder.setPosition(0);
        // }
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