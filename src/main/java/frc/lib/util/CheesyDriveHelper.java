package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that the "turning" stick controls the curvature
 * of the robot's path rather than its rate of heading change. This helps make the robot more controllable at high
 * speeds. Also handles the robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
 * turn-in-place maneuvers.
 */
public class CheesyDriveHelper {

    private static final double kThrottleDeadband = 0.02;
    private static final double kWheelDeadband = 0.02;

    // These factor determine how fast the wheel traverses the "non linear" sine curve.
    private static double kHighWheelNonLinearity = 0.7;
    private static double kLowWheelNonLinearity = 0.5;

    private static double kHighNegInertiaScalar = 2; //1.8

    private static double kLowNegInertiaThreshold = 0.65;
    private static double kLowNegInertiaTurnScalar = 3.5;
    private static double kLowNegInertiaCloseScalar = 4.0;
    private static double kLowNegInertiaFarScalar = 5.0;

    private static double kHighSensitivity = 0.7;
    private static double kLowSensitiity = 0.85;

    private static double kQuickStopDeadband = 0.5;
    private static double kQuickStopWeight = 0.15;
    private static double kQuickStopScalar = 6.0;

    private double mOldWheel = 0.0;
    private double mQuickStopAccumlator = 0.0;
    private double mNegInertiaAccumlator = 0.0;

    public CheesyDriveHelper(){
        SmartDashboard.putNumber("kHighWheelNonLinearity",kHighWheelNonLinearity);
        SmartDashboard.putNumber("kLowWheelNonLinearity",kLowWheelNonLinearity);
        SmartDashboard.putNumber("kHighNegInertiaScalar",kHighNegInertiaScalar);
        SmartDashboard.putNumber("kLowNegInertiaThreshold",kLowNegInertiaThreshold);
        SmartDashboard.putNumber("kLowNegInertiaCloseScalar",kLowNegInertiaCloseScalar);
        SmartDashboard.putNumber("kLowNegInertiaFarScalar",kLowNegInertiaFarScalar);
        SmartDashboard.putNumber("kHighSensitivity",kHighSensitivity);
        SmartDashboard.putNumber("kLowSensitiity",kLowSensitiity);
        SmartDashboard.putNumber("kQuickStopDeadband",kQuickStopDeadband);
        SmartDashboard.putNumber("kQuickStopWeight",kQuickStopWeight);
        SmartDashboard.putNumber("kQuickStopScalar",kQuickStopScalar);
    }

    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn,
                                   boolean isHighGear) {
        kHighWheelNonLinearity = SmartDashboard.getNumber("kHighWheelNonLinearity",kHighWheelNonLinearity);
        kLowWheelNonLinearity = SmartDashboard.getNumber("kLowWheelNonLinearity",kLowWheelNonLinearity);
        kHighNegInertiaScalar = SmartDashboard.getNumber("kHighNegInertiaScalar",kHighNegInertiaScalar);
        kLowNegInertiaThreshold = SmartDashboard.getNumber("kLowNegInertiaThreshold",kLowNegInertiaThreshold);
        kLowNegInertiaCloseScalar = SmartDashboard.getNumber("kLowNegInertiaCloseScalar",kLowNegInertiaCloseScalar);
        kLowNegInertiaFarScalar = SmartDashboard.getNumber("kLowNegInertiaFarScalar",kLowNegInertiaFarScalar);
        kHighSensitivity = SmartDashboard.getNumber("kHighSensitivity",kHighSensitivity);
        kLowSensitiity = SmartDashboard.getNumber("kLowSensitiity",kLowSensitiity);
        kQuickStopDeadband = SmartDashboard.getNumber("kQuickStopDeadband",kQuickStopDeadband);
        kQuickStopWeight = SmartDashboard.getNumber("kQuickStopWeight",kQuickStopWeight);
        kQuickStopScalar = SmartDashboard.getNumber("kQuickStopScalar",kQuickStopScalar);
        wheel = handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = kHighWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = kLowWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = kHighNegInertiaScalar;
            sensitivity = kHighSensitivity;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = kLowNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(wheel) > kLowNegInertiaThreshold) {
                    negInertiaScalar = kLowNegInertiaFarScalar;
                } else {
                    negInertiaScalar = kLowNegInertiaCloseScalar;
                }
            }
            sensitivity = kLowSensitiity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
                        + alpha * Util.limit(wheel, 1.0) * kQuickStopScalar;
            }
            overPower = 0.5;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        return new DriveSignal(leftPwm, rightPwm);
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}