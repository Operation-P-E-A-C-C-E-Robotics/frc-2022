// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.MAX_OUTPUT;
import static frc.robot.Constants.TurretConstants.MIN_OUTPUT;
import static frc.robot.Constants.TurretConstants.TURRET_CONTROLLER_PORT;
import static frc.robot.Constants.TurretConstants.kD;
import static frc.robot.Constants.TurretConstants.kFF;
import static frc.robot.Constants.TurretConstants.kI;
import static frc.robot.Constants.TurretConstants.kIz;
import static frc.robot.Constants.TurretConstants.kP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;


public class OldTurret extends SubsystemBase {
    private final CANSparkMax turretMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;
    // private final DigitalInput zeroSwitch;

    private double setpoint = 0;
    // private final double TURRET_RATIO = (1/20) * (1/11); //todo get ratio
    private final double MOTOR_ROTS_PER_TURRET_ROT = 242;




    /** Creates a new Shooter. */
    public OldTurret() {
        turretMotor = new CANSparkMax(TURRET_CONTROLLER_PORT, MotorType.kBrushless);
        
        encoder = turretMotor.getEncoder();
        
        pidController = turretMotor.getPIDController();
        
        turretMotor.setInverted(false);
        turretMotor.setSmartCurrentLimit(10);

        new GainSetter(pidController)
        .ff(kFF)
        .p(kP)
        .i(kI)
        .d(kD)
        .iz(kIz)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT);
    
        
    }

    public GainSetter getGainSetter(){
        return new GainSetter(pidController);
    }

    public void setTurretRotations(double rotations){
        //limit rotations to += 180 degrees, and if value greater than that, rotate around the other way
        // rotations = ((rotations + 0.5) % 1) - 0.5;
        rotations = Util.limit(rotations, 0.5);
        rotations *= MOTOR_ROTS_PER_TURRET_ROT; //convert from motor rotations to turret rotations
        pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        setpoint = rotations;
    }

    /**
     * UNTESTED turret velocity control
     */
    public void setTurretVelocity(double velocity){
        pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * set the turret motor power
     * @param speed the speed (-1 to 1)
     */
    public void setTurretPercent(Double speed) {
        turretMotor.set(speed);
    }

    /**
     * get turret position in rotations
     */
    public double getPosition(){
        return encoder.getPosition() / MOTOR_ROTS_PER_TURRET_ROT; //should it be / or *??
    }

    /**
     * zero the turret encoder
     */
    public void zero(){
        encoder.setPosition(0);
    }

    /**
     * tell if the turret is ready to shoot
     * @return whether the turret is in position
     */
    public boolean ready(){
        return (Math.abs(setpoint) - Math.abs(encoder.getPosition())) < 3;
    }

    @Override
    public void periodic() {
        
    }

    /**
     * allows nice syntax for setting spark max pid gains
     */
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