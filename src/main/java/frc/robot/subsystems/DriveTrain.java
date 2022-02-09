// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;
import static frc.robot.Constants.DriveTrain.*;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonFX leftMasterController = new WPI_TalonFX(LEFT_MASTER_PORT);
    private final WPI_TalonFX leftSlaveController = new WPI_TalonFX(LEFT_SLAVE_PORT);
    private final WPI_TalonFX rightSlaveController = new WPI_TalonFX(RIGHT_SLAVE_PORT);
    private final WPI_TalonFX rightMasterController = new WPI_TalonFX(RIGHT_MASTER_PORT);
    private final DifferentialDrive dDrive = new DifferentialDrive(leftMasterController, rightMasterController);

    private final double wheelCircumference = Math.PI * WHEEL_DIAMETER_METERS;
    //shifter stuff
    //   public DoubleSolenoid shiftSolenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 0,0);
    @SuppressWarnings("FieldMayBeFinal")
    private Gear _gear = Gear.LOW_GEAR;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        setNeutralModes(NeutralMode.Coast);

        leftSlaveController.follow(leftMasterController);

    // rightMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 35, 2));
    // leftMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 35, 2));

    // rightMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 0));
    // leftMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 0));

        //depending on gearboxes, motors could end up fighting. if so, change.
        leftMasterController.setInverted(false);
        rightMasterController.setInverted(true);
        leftSlaveController.setInverted(InvertType.FollowMaster);
        rightSlaveController.setInverted(InvertType.FollowMaster);

        leftMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 35, 2));
        rightMasterController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 35, 2));

    }

    /**
     * shift the drivetrain into high, low, or MAYBE neutral gear
     * @param gear the drivetrain.gear to shift into
     */
    //     public void shift(Gear gear) {
    //       if(gear == Gear.HIGH_GEAR){
    //           shiftSolenoid.set(Value.kForward);
    //       }
    //       if(gear == Gear.LOW_GEAR){
    //           shiftSolenoid.set(Value.kReverse);
    //       }
    //       if(gear == Gear.NEUTRAL){
    //           shiftSolenoid.set(Value.kOff);
    //       }
    //       _gear = gear;
    //   }

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
            leftMasterController.getSelectedSensorVelocity(0) / DRIVE_ENCODER_CPR / DRIVE_HIGH_GEAR_RATIO * wheelCircumference, rightMasterController.getSelectedSensorVelocity(0) / DRIVE_ENCODER_CPR / DRIVE_HIGH_GEAR_RATIO * wheelCircumference
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
        SmartDashboard.putNumber("drivetrain l1 temp", leftMasterController.getTemperature());
        SmartDashboard.putNumber("drivetrain l2 temp", leftSlaveController.getTemperature());
        SmartDashboard.putNumber("drivetrain r1 temp", rightMasterController.getTemperature());
        SmartDashboard.putNumber("drivetrain r2 temp", rightSlaveController.getTemperature());
        SmartDashboard.putNumber("drivetrain left current", leftMasterController.getSupplyCurrent() + leftSlaveController.getSupplyCurrent());
        SmartDashboard.putNumber("drivetrain right current", rightMasterController.getSupplyCurrent() + rightSlaveController.getSupplyCurrent());
        dDrive.feed();
    }

    public enum Gear {
        HIGH_GEAR,
        LOW_GEAR,
        NEUTRAL
    }
}