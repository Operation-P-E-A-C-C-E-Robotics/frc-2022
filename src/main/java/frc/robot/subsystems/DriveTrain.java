// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;
import static frc.robot.Constants.DriveTrainConstants.*;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonFX leftMasterController = new WPI_TalonFX(LEFT_MASTER_PORT);
    private final WPI_TalonFX leftSlaveController = new WPI_TalonFX(LEFT_SLAVE_PORT);
    private final WPI_TalonFX rightSlaveController = new WPI_TalonFX(RIGHT_SLAVE_PORT);
    private final WPI_TalonFX rightMasterController = new WPI_TalonFX(RIGHT_MASTER_PORT);
    private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0,1);
    
    private final DifferentialDrive dDrive = new DifferentialDrive(leftMasterController, rightMasterController);
    private final PIDController lPid = new PIDController(high_kP, high_kI, high_kD);
    private final PIDController rPid = new PIDController(high_kP, high_kI, high_kD);
    private final SimpleMotorFeedforward highGearFeedforward = new SimpleMotorFeedforward(high_kS, high_kV, high_kA);
    private final SimpleMotorFeedforward lowGearFeedforward = new SimpleMotorFeedforward(low_kS, low_kV, low_kA);

    //DRIVE RATIOS 54:30, 64:20

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

        leftMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 10));
        rightMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 10));
    
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
        //SmartDashboard.putNumber("DT Left Speed", lSpeed);
        //SmartDashboard.putNumber("DT Right Speed", rSpeed);
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

    public void velocityDrive(double lSpeed, double rSpeed){
        if(getGear() == Gear.HIGH_GEAR){
            lPid.setP(high_kP);
            lPid.setI(high_kI);
            lPid.setD(high_kD);
            rPid.setP(high_kP);
            rPid.setI(high_kI);
            rPid.setD(high_kD);
            leftMasterController.setVoltage(highGearFeedforward.calculate(lSpeed)
                + lPid.calculate(leftMasterController.getSelectedSensorVelocity(), lSpeed));
            rightMasterController.setVoltage(highGearFeedforward.calculate(rSpeed)
                + rPid.calculate(rightMasterController.getSelectedSensorVelocity(), rSpeed));
        } else {
            lPid.setP(low_kP);
            lPid.setI(low_kI);
            lPid.setD(low_kD);
            rPid.setP(low_kP);
            rPid.setI(low_kI);
            rPid.setD(low_kD);
            leftMasterController.setVoltage(lowGearFeedforward.calculate(lSpeed)
                + lPid.calculate(leftMasterController.getSelectedSensorVelocity(), lSpeed));
            rightMasterController.setVoltage(lowGearFeedforward.calculate(rSpeed)
                + rPid.calculate(rightMasterController.getSelectedSensorVelocity(), rSpeed));
        }
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
    public double getAverageEncoderMeters() {
        return (lEncoderPosition() + rEncoderPosition()) / 2.0;
    }

    public double getAverageEncoderCounts() {
        return (leftMasterController.getSelectedSensorPosition() + rightMasterController.getSelectedSensorPosition()) / 2;
    }

    public double getLeftEncoderSimple() {
        return leftMasterController.getSelectedSensorPosition();
    }

    public double getRightEncoderSimple() {
        return rightMasterController.getSelectedSensorPosition();
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

    double autoShiftThreshold = 100; //TODO TODO TODO

    @Override
    public void periodic() {
        // if(rightMasterController.getSelectedSensorVelocity() > autoShiftThreshold ||
        //     leftMasterController.getSelectedSensorVelocity() > autoShiftThreshold){
        //     shift(Gear.HIGH_GEAR);
        // } else {
        //     shift(Gear.LOW_GEAR);
        // }
        // SmartDashboard.putNumber("drivetrain left current", leftMasterController.getSupplyCurrent() + leftSlaveController.getSupplyCurrent());
        // SmartDashboard.putNumber("drivetrain right current", rightMasterController.getSupplyCurrent() + rightSlaveController.getSupplyCurrent());
        dDrive.feed();

        SmartDashboard.putNumber("DT Avg Enc", getAverageEncoderCounts());
    
        // high_kS = SmartDashboard.getNumber("high ks", high_kS);
        // high_kV = SmartDashboard.getNumber("high kv", high_kV);
        // high_kA = SmartDashboard.getNumber("high ka", high_kA);
        // high_kP = SmartDashboard.getNumber("high kp", high_kP);
        // high_kI = SmartDashboard.getNumber("high ki", high_kI);
        // high_kD = SmartDashboard.getNumber("high kd", high_kD);
        // low_kS = SmartDashboard.getNumber("low ks", low_kS);
        // low_kV = SmartDashboard.getNumber("low kv", low_kV);
        // low_kA = SmartDashboard.getNumber("low ka", low_kA);
        // low_kP = SmartDashboard.getNumber("low kp", low_kP);
        // low_kI = SmartDashboard.getNumber("low ki", low_kI);
        // low_kD = SmartDashboard.getNumber("low kd", low_kD);
    }

    public enum Gear {
        HIGH_GEAR,
        LOW_GEAR,
        NEUTRAL
    }
}