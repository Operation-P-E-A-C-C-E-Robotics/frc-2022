// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.DRIVE_ENCODER_CPR;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_HIGH_GEAR_RATIO;
import static frc.robot.Constants.DriveTrainConstants.LEFT_MASTER_PORT;
import static frc.robot.Constants.DriveTrainConstants.LEFT_SLAVE_PORT;
import static frc.robot.Constants.DriveTrainConstants.RIGHT_MASTER_PORT;
import static frc.robot.Constants.DriveTrainConstants.RIGHT_SLAVE_PORT;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_DIAMETER_METERS;
import static frc.robot.Constants.DriveTrainConstants.high_kA;
import static frc.robot.Constants.DriveTrainConstants.high_kD;
import static frc.robot.Constants.DriveTrainConstants.high_kI;
import static frc.robot.Constants.DriveTrainConstants.high_kP;
import static frc.robot.Constants.DriveTrainConstants.high_kS;
import static frc.robot.Constants.DriveTrainConstants.high_kV;
import static frc.robot.Constants.DriveTrainConstants.low_kA;
import static frc.robot.Constants.DriveTrainConstants.low_kD;
import static frc.robot.Constants.DriveTrainConstants.low_kI;
import static frc.robot.Constants.DriveTrainConstants.low_kP;
import static frc.robot.Constants.DriveTrainConstants.low_kS;
import static frc.robot.Constants.DriveTrainConstants.low_kV;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonFX leftMasterController = new WPI_TalonFX(LEFT_MASTER_PORT);
    private final WPI_TalonFX leftSlaveController = new WPI_TalonFX(LEFT_SLAVE_PORT);
    private final WPI_TalonFX rightSlaveController = new WPI_TalonFX(RIGHT_SLAVE_PORT);
    private final WPI_TalonFX rightMasterController = new WPI_TalonFX(RIGHT_MASTER_PORT);
    private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0,1);
    
    private final DifferentialDrive dDrive = new DifferentialDrive(leftMasterController, rightMasterController);

    private Gear _gear = Gear.LOW_GEAR;

    private final double wheelCircumference = Math.PI * WHEEL_DIAMETER_METERS;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        setNeutralModes(NeutralMode.Brake);

        leftSlaveController.follow(leftMasterController);
        rightSlaveController.follow(rightMasterController);


        //depending on gearboxes, motors could end up fighting. if so, change.
        leftMasterController.setInverted(true);
        rightMasterController.setInverted(false);
        leftSlaveController.setInverted(InvertType.FollowMaster);
        rightSlaveController.setInverted(InvertType.FollowMaster);

        leftMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 60, 10));
        rightMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 60, 10));
    
        // SmartDashboard.putNumber("high ks", high_kS);
        // SmartDashboard.putNumber("high kv", high_kV);
        // SmartDashboard.putNumber("high ka", high_kA);
        // SmartDashboard.putNumber("high kp", high_kP);
        // SmartDashboard.putNumber("high ki", high_kI);
        // SmartDashboard.putNumber("high kd", high_kD);
        // SmartDashboard.putNumber("low ks", low_kS);
        // SmartDashboard.putNumber("low kv", low_kV);
        // SmartDashboard.putNumber("low ka", low_kA);
        // SmartDashboard.putNumber("low kp", low_kP);
        // SmartDashboard.putNumber("low ki", low_kI);
        // SmartDashboard.putNumber("low kd", low_kD);
    }

    /**
     * shift the drivetrain into high, low, or MAYBE neutral gear
     * @param gear the drivetrain.gear to shift into
     */
    public void shift(Gear gear) {
        if(gear == Gear.HIGH_GEAR){
            shiftSolenoid.set(Value.kForward);
        }
        if(gear == Gear.LOW_GEAR){
            shiftSolenoid.set(Value.kReverse);
        }
        if(gear == Gear.NEUTRAL){
            shiftSolenoid.set(Value.kOff);
        }
        _gear = gear;
    }

    /**
     * get the current gear of the drivetrain gearboxes
     * @return
     */
    public Gear getGear() {
        return _gear;
    }

    /**
     * set left and right speeds as a percentage of full power
     * @param rSpeed right side power percentage
     * @param lSpeed left side power percentage
     */
    public void percentDrive(double rSpeed, double lSpeed) {
        leftMasterController.set(ControlMode.PercentOutput, lSpeed);
        rightMasterController.set(ControlMode.PercentOutput, rSpeed);
        dDrive.feed(); //keep the wpilib arcade drive that we need for auto from freaking out
    }

    /**
     * use the wpilib arcade drive like a crazy person
     * @param spd forward speed
     * @param rot rotation left-right duh
     */
    public void arcadeDrive(double spd, double rot) {
        dDrive.arcadeDrive(spd, rot);
    }

    /**
     * takes a lib.DriveSiganl from custom drivetrain code bcuz i am not a crazy person
     * @param signal the DriveSignal to set the motors to to.
     */
    public void arcadeDrive(DriveSignal signal) {
        leftMasterController.set(signal.getLeft());
        rightMasterController.set(signal.getRight());
        dDrive.feed();
    }

    /**
     * drive using left and right motor voltages.
     * This is required for auto
     * @param lVolts voltage to feed to the left motor
     * @param rVolts voltage to feed to the right motor
     */
    public void voltageDrive(double lVolts, double rVolts) {
        leftMasterController.setVoltage(lVolts);
        rightMasterController.setVoltage(rVolts);
        dDrive.feed();
    }

    /**
     * get wheel velocity (for auto)
     * @return wplilib differentialdrivewheelspeeds containing velocity info
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftMasterController.getSelectedSensorVelocity(0) / DRIVE_ENCODER_CPR / DRIVE_HIGH_GEAR_RATIO * wheelCircumference, 
            rightMasterController.getSelectedSensorVelocity(0) / DRIVE_ENCODER_CPR / DRIVE_HIGH_GEAR_RATIO * wheelCircumference
        );
    }

    /**
     * reset the falcon encoder positions back to 0
     */
    public void resetEncoders() {
        leftMasterController.setSelectedSensorPosition(0);
        rightMasterController.setSelectedSensorPosition(0);
    }

    /**
     * @return average of left and right encoder positions
     */
    public double getAverageEncoderDistance() {
        return (lEncoderPosition() + rEncoderPosition()) / 2.0;
    }

    /**
     * set maximum voltage to motors.
     * for auto, or when somebody else is trying out for drive team
     * @param maxVolts the voltage limit
     */
    public void setVoltageConstraint(double maxVolts) {
        leftMasterController.configVoltageCompSaturation(maxVolts);
        rightMasterController.configVoltageCompSaturation(maxVolts);
    }

    /**
     * turn off the voltage constraint
     * @param isEnabled enable or disable the voltage constraint
     */
    public void voltageConstraintEnabled(boolean isEnabled) {
        leftMasterController.enableVoltageCompensation(isEnabled);
        rightMasterController.enableVoltageCompensation(isEnabled);
    }

    /**
     * @return position of left side encoder
     */
    public double lEncoderPosition() {
        return leftMasterController.getSelectedSensorPosition() / DRIVE_ENCODER_CPR / DRIVE_HIGH_GEAR_RATIO * wheelCircumference;
    }
    /**
     * @return position of right side encoder
     */
    public double rEncoderPosition() {
        return rightMasterController.getSelectedSensorPosition() / DRIVE_ENCODER_CPR / DRIVE_HIGH_GEAR_RATIO * wheelCircumference;
    }

    /**
     * set the neutral mode of left+right motors at same time
     * This way, some doosh like me doesn't only turn off brake for one side only
     * @param mode Neutral mode to set the motors to
     */
    public void setNeutralModes(NeutralMode mode) {
        leftMasterController.setNeutralMode(mode);
        rightMasterController.setNeutralMode(mode);
    }
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("drivetrain left current", leftMasterController.getSupplyCurrent() + leftSlaveController.getSupplyCurrent());
        // SmartDashboard.putNumber("drivetrain right current", rightMasterController.getSupplyCurrent() + rightSlaveController.getSupplyCurrent());
        dDrive.feed();
    }

    public enum Gear {
        HIGH_GEAR,
        LOW_GEAR,
        NEUTRAL
    }
}