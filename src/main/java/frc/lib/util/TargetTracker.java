package frc.lib.util;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.PointTracker;
import frc.lib.math.Sequencer;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.subsystems.Turret;

public class TargetTracker {
    private Limelight limelight;
    private Pigeon pigeon;
    private DifferentialDriveOdometry odometry;
    private Turret turret;
    private Translation2d targetOffset;
    private Pose2d odometryOffset;

    private PointTracker fieldRelativeTargetFromRobotCombined,
                         robotTranslation,
                         fieldRelativeTargetFromRobotFromLimelight,
                         fieldRelativeTargetFromRobotFromDrivetrain;

    private double[] limelightTargetAreaBuffer = new double[3];
    private PointTracker limelightTargetCenterTracker;

    private TrackingMode mode = TrackingMode.NORMAL;
    private ErrorMode error = ErrorMode.NORMAL;
    private ManualBackupMode backup = ManualBackupMode.NONE;

    //TODO get good values; filters out outlier targets
    //by making sure they're in the expected range of the odometry
    private final double allowedDistanceDifference = 1, //meters
                         allowedAngleDifference = 30, //degrees
                         areaAllowedVelocity = 10, //% of image per update
                         areaAllowedAcceleration = 3, //% of image per update
                         targetSmoothingFactor = 10, //craycray maths, higher = more smoothing
                         targetSmoothingCeiling = 0; //ceiling on target jerk from logged points (0 forces constant acceleration)
    

    private Joystick joystick;
    Translation2d manualOffset = new Translation2d(0,0);
    private final double manualSensitivity = 0.05;
    
    /**
     * combines odometry and limelight values to track a point on the field.
     * applies filtering and smoothing to limelight targets based on odometry expected values
     * @param limelight a Limelight
     * @param turret a Turret. must have a getAngle() function that returns a Rotation2d with the turret angle, ccw pos
     * @param pigeon a Pigeon.
     * @param odometry a DifferentialDriveOdometry
     * @param odometryOffset the offset of the odometry values from the actual robot position. will be changed in code
     * @param targetOffset the location of the target on the field.
     */
    public TargetTracker(Limelight limelight, Turret turret, Pigeon pigeon, DifferentialDriveOdometry odometry, Pose2d odometryOffset, Translation2d targetOffset, Joystick operatorJoystick){
        this.limelight = limelight;
        this.pigeon = pigeon;
        this.odometry = odometry;
        this.turret = turret;
        this.targetOffset = targetOffset;
        this.odometryOffset = odometryOffset;
        this.joystick = operatorJoystick;
        fieldRelativeTargetFromRobotCombined = new PointTracker(3);
        robotTranslation = new PointTracker(1);
        fieldRelativeTargetFromRobotFromLimelight = new PointTracker(1);
        fieldRelativeTargetFromRobotFromDrivetrain = new PointTracker(1);

        limelightTargetAreaBuffer = Util.zeros(limelightTargetAreaBuffer);
        limelightTargetCenterTracker = new PointTracker(3);
        limelightTargetCenterTracker = new PointTracker(3);
    }

    /**
     * get the target angle relative to the field (a.k.a the pigeon's zero point)
     * @return a Rotation2d with the angle of the target
     */
    public Rotation2d getTargetAngle(){
        Translation2d targetTranslation = fieldRelativeTargetFromRobotCombined.getTranslation();
        if(backup == ManualBackupMode.MANUAL_OFFSETS) targetTranslation.plus(manualOffset);
        if(backup == ManualBackupMode.FULL_MANUAL_TARGET_POSITION ||
            backup == ManualBackupMode.FULL_MANUAL_UNTIL_LIMELIGHT)
                targetTranslation = manualOffset;
        if(mode == TrackingMode.EMERGENCY_LIMELIGHT_RAW) return fieldRelativeTargetAngleFromLimelight();
        return new Rotation2d(PointTracker.fromTranslation(targetTranslation).p());
    }

    /**
     * get the distance from the robot to the target
     * @return the target distance in meters (as long as everything else is meters)
     */
    public double getTargetDistance(){
        return fieldRelativeTargetFromRobotCombined.p();
    }

    /**
     * update the tracking
     * @param mode which things to use while tracking
     * @param aiming whether or not we're actively aiming. THIS MUST BE SET CORRECTLY.
     */
    public void update(TrackingMode mode, boolean aiming){
        this.mode = mode;
        update(aiming);
    }

    /**
     * update the tracking
     * @param aiming whether or not we're actively aiming. THIS MUST BE SET CORRECTLY.
     */
    public void update(boolean aiming){
        Pose2d robotPose;
        Rotation2d limelightTargetAngle;
        Translation2d targetFromRobot;
        double targetDistance;

        this.aiming = aiming;

        //recompute all the points with new cycle, instead of re-returning computations
        recomputeTIB = true;
        recomputeFLT = true;

        //reset in case both numbers aren't available, so it won't 'stick'
        //when the limelight doesn't have a target
        differenceBetweenLimelightAndOdometry = 0;

        checkForErrors();
        handleError();

        manualOffset.plus(new Translation2d(
            joystick.getX() * manualSensitivity,
            joystick.getY() * manualSensitivity
        ));

        if(!aiming){
            aimStartTime = Timer.getFPGATimestamp();
        }

        //update area buffer
        Util.shiftLeft(limelightTargetAreaBuffer, limelight.getTargetArea());

        //update robot pose/translation
        robotPose = odometry.getPoseMeters().relativeTo(odometryOffset);//TODO this might have to be fixed
        robotTranslation.setTranslation(robotPose.getTranslation());

        //update limelight target position
        limelightTargetAngle = fieldRelativeTargetAngleFromLimelight();
        targetDistance = limelightTargetDistance();
        fieldRelativeTargetFromRobotFromLimelight.pr(limelightTargetAngle.getRadians(), targetDistance);

        //update drivetrain target position
        targetFromRobot = targetOffset.minus(robotPose.getTranslation());
        fieldRelativeTargetFromRobotFromDrivetrain.setTranslation(targetFromRobot);
        
        if(mode == TrackingMode.LIMELIGHT_ONLY){
            fieldRelativeTargetFromRobotCombined = fieldRelativeTargetFromRobotFromLimelight;
        } else {
            //if limelight has no target, only use drivetrain (unless it's set to not use odometry)
            fieldRelativeTargetFromRobotCombined = fieldRelativeTargetFromRobotFromDrivetrain;
        }

        //only continue if there is a valid target and we are using the limelight
        if(!targetInBounds() || mode == TrackingMode.ODOMETRY_ONLY) return;
        
        //get the difference between the limelight odometry and the regular odometry (for error detection)
        PointTracker difference = new PointTracker(1).xy(
            Util.difference(robotPose.getX(),odometryFromLimelight().getX()),
            Util.difference(robotPose.getY(),odometryFromLimelight().getY())
        );

        differenceBetweenLimelightAndOdometry = difference.r();

        //update odometry from limelight odometry
        odometryOffset = robotPose.relativeTo(odometryFromLimelight());
    }

    private final double allowableTimeToSeeTarget = 5; //seconds
    // private final double targetCenteredThreshold = 4; //degrees
    private final int numberOfTargetLossesToTrip = 10; //times
    private final double allowableDfferenceBetweenLimelightAndOdometry = 1;//meters

    private boolean aiming = false;
    // private boolean limelightShouldSeeTarget = false;
    private boolean limelightHasSeenTarget = false;
    private int limelightTargetSightingsSinceLoss = 0;
    // private int limelightTargetLossesSinceLastSighting = 0;
    private int limelightLossesSinceTargetFound = 0;
    private double differenceBetweenLimelightAndOdometry = 0;
    private double aimStartTime = 0;

    /**
     * check to make sure everything is working smoothly!
     */
    private void checkForErrors(){
        double aimingTime = Timer.getFPGATimestamp() - aimStartTime;
        boolean hasTarget = limelight.hasTarget() == 1;

        //use pigeon to try and detect possible bumps (to give some manual control to drivers)
        if(pigeon.wasBumped()) error = ErrorMode.SUSPECTED_BUMP;

        /**
         * make sure the limelight and odometry are lining up as expected.
         * if they don't we'll give some control to the drivers to make sure
         * things go smoothly
         */
        if(differenceBetweenLimelightAndOdometry > allowableDfferenceBetweenLimelightAndOdometry)
            error = ErrorMode.UNEXPECTED_DIFFERENCE_BETWEEN_ODOMETRY_AND_LIMELIGHT;

        
        if(!aiming) return; //only continue if we're trying to aim
        //active aiming errors:

        if(hasTarget) {
            limelightTargetSightingsSinceLoss++;
            // limelightTargetLossesSinceLastSighting = 0;
        }else {
            // limelightTargetLossesSinceLastSighting++;
            limelightTargetSightingsSinceLoss = 0;
        }

        if (limelightTargetSightingsSinceLoss >= 5){
            limelightHasSeenTarget = true;
        }

        if(limelightHasSeenTarget && !hasTarget) limelightLossesSinceTargetFound++;

        //if the limelight repeatedly loses the target, then we give control to the drivers
        if(limelightLossesSinceTargetFound >= numberOfTargetLossesToTrip) error = ErrorMode.LIMELIGHT_CANT_HOLD_TARGET;

        //if the limelight never finds the target, then we give control to the drivers
        //until the limelight sees a target (in case the odometry gets messed up)
        if(aimingTime > allowableTimeToSeeTarget && !limelightHasSeenTarget) error = ErrorMode.LIMELIGHT_TARGET_NOT_SEEN;

        //if the limelight has found the target, then disable this manual
        //until limelight backup.
        if(backup == ManualBackupMode.FULL_MANUAL_UNTIL_LIMELIGHT &&
            limelightHasSeenTarget)
                backup = ManualBackupMode.NONE;

        //TODO when have result functions, do NOT_REACHING_TARGET
        //check whether the turret is angled close enough to the correct
        //direction.
    }

    /**
     * sets the tracking mode and backup mode to handle error modes.
     */
    private void handleError(){
        switch(error){
            case NORMAL:
                mode = TrackingMode.NORMAL;
                backup = ManualBackupMode.NONE;
                break;
            
            case LIMELIGHT_TARGET_NOT_SEEN:
                mode = TrackingMode.EMERGENCY_LIMELIGHT_RAW;
                backup = ManualBackupMode.FULL_MANUAL_UNTIL_LIMELIGHT;
                break;
            
            case UNEXPECTED_DIFFERENCE_BETWEEN_ODOMETRY_AND_LIMELIGHT:
                mode = TrackingMode.LIMELIGHT_ONLY;
                backup = ManualBackupMode.FULL_MANUAL_UNTIL_LIMELIGHT;
                break;
            case LIMELIGHT_CANT_HOLD_TARGET:
                mode = TrackingMode.ODOMETRY_ONLY;
                backup = ManualBackupMode.MANUAL_OFFSETS;
                break;
            case SUSPECTED_BUMP:
                backup = ManualBackupMode.FULL_MANUAL_UNTIL_LIMELIGHT;
                break;
            case NOT_REACHING_TARGET:
                mode = TrackingMode.EMERGENCY_LIMELIGHT_RAW;
                backup = ManualBackupMode.FULL_MANUAL_UNTIL_LIMELIGHT;
                break;
        }
    }

    /**
     * get the robot position on the field using the limelight
     * @return the robots Pose
     */
    private Pose2d odometryFromLimelight(){
        Translation2d targetTranslation = filterLimelightTarget().getTranslation();
        Translation2d robotTranslationFromTarget = new Translation2d(0,0).minus(targetTranslation);
        return new Pose2d(targetTranslation.minus(robotTranslationFromTarget), Rotation2d.fromDegrees(pigeon.getHeading()));
    }

    /**
     * get the distance to the target from the limelight.
     * unlike the function in Limelight.java, this one
     * uses the fancy smoothing
     * @return the distance to the target in meters
     */
    private double limelightTargetDistance(){
        double distance = 0;
        double targetHeight = limelight.getTargetHeight();
        double cameraHeight = limelight.getCameraHeight();
        double cameraAngle = limelight.getCameraAngle();
        distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle) + Math.toRadians(filterLimelightTarget().y()));
        return distance;
    }

    /**
     * get the limelight target X offset with smoothing.
     * @return the limelight offset, in probably degrees lol
     */
    private double limelightTargetOffset(){
        return filterLimelightTarget().x();
    }

    private boolean recomputeTIB = true;
    private boolean _targetInBounds;

    /**
     * get whether the target meets the filtering criteria
     * @return boolean true if the target is valid
     */
    private boolean targetInBounds(){
        //save computing power by only computing once per cycle
        if(recomputeTIB) _targetInBounds = _targetInBounds();
        recomputeTIB = false;
        return _targetInBounds;
    }

    private boolean _targetInBounds(){
        Rotation2d expectedRotationFromHeading, targetOffsetFromTurret;
        double expectedLimelightXAngle, expectedLimelightDistance;
        double[] computed;

        if(limelight.hasTarget() == 0) return false; //no target = out of bounds
    
        //disable boundary checking if mode says to
        if(mode != TrackingMode.DISABLE_TARGET_BOUNDARY){
            //make sure in allowed range of expected X angle
            expectedRotationFromHeading = getRobotRelativeAngleToPointFromFieldRelativePoint(odometry.getPoseMeters(), fieldRelativeTargetFromRobotCombined.getTranslation());
            targetOffsetFromTurret = expectedRotationFromHeading.minus(turret.getAngle());
            expectedLimelightXAngle = targetOffsetFromTurret.getDegrees();
            if(!Util.inRange(Util.difference(expectedLimelightXAngle, limelight.getTargetOffsetX()), allowedAngleDifference)) return false;
        
            //make sure within allowed range of expected distance
            expectedLimelightDistance = fieldRelativeTargetFromRobotCombined.p();
            if(!Util.inRange(Util.difference(expectedLimelightDistance, limelight.getTargetDistance()), allowedDistanceDifference)) return false;
        }

        //make sure target within area fluctuation bounds
        computed = Sequencer.compute(limelightTargetAreaBuffer);
        if(!Util.inRange(computed[1], areaAllowedVelocity)) return false;
        if(!Util.inRange(computed[2], areaAllowedAcceleration)) return false;

        return true;
    }

    private boolean recomputeFLT = true;
    private PointTracker _filterLimelightTarget;

    /**
     * apply filters to the limelight's target center,
     * for smoothing and to reject erronious targets while the odometry is
     * getting it pointed towards the target
     * @return 
     */
    private PointTracker filterLimelightTarget(){
        //save computing power by only computing once per cycle
        if(recomputeFLT) _filterLimelightTarget = _filterLimelightTarget();
        recomputeFLT = false;
        return _filterLimelightTarget;
    }

    private PointTracker _filterLimelightTarget(){
        if(!targetInBounds()){
            //just use prediction if the limelight target is erronious
            limelightTargetCenterTracker = limelightTargetCenterTracker.getFuture(1);
            return limelightTargetCenterTracker.smooth(targetSmoothingFactor, targetSmoothingCeiling);
        }

        //update and return smoothed target center (unless smoothing disabled)
        limelightTargetCenterTracker.xy(limelight.getTargetOffsetX(), limelight.getTargetOffsetY());
        return mode == TrackingMode.DISABLE_TARGET_SMOOTHING ? 
            limelightTargetCenterTracker : 
            limelightTargetCenterTracker.smooth(targetSmoothingFactor, targetSmoothingCeiling);
    }

    /**
     * get the angle from robot's heading to a point
     * @param robotPose robot's pose
     * @param pointTranslation point to get angle for
     * @return Rotation2d of the offset from robot's forward, ccw positive
     */
    private Rotation2d getRobotRelativeAngleToPointFromFieldRelativePoint(Pose2d robotPose, Translation2d pointTranslation){
        Translation2d targetTranslationFromRobot = pointTranslation.minus(robotPose.getTranslation());
        Rotation2d targetRotationFromRobotFieldRelative = new Rotation2d(PointTracker.fromTranslation(targetTranslationFromRobot).p());
        Rotation2d robotHeading = robotPose.getRotation();
        return targetRotationFromRobotFieldRelative.minus(robotHeading);
    }

    /**
     * get the limelight's target offset relative to the absolute zero
     * @return rotation2d with target offset from absolute zero, ccw positive
     */
    private Rotation2d fieldRelativeTargetAngleFromLimelight(){
        //odometry uses counterclockwise-positive rotation so do the same here
        Rotation2d xRotationsTurretRelative = Rotation2d.fromDegrees(-limelightTargetOffset());
        Rotation2d xRotationsRobotRelative = xRotationsTurretRelative.plus(turret.getAngle());
        return Rotation2d.fromDegrees(pigeon.getHeading()).plus(xRotationsRobotRelative);
    }

    public enum TrackingMode{
        NORMAL,
        LIMELIGHT_ONLY,
        ODOMETRY_ONLY,
        DISABLE_TARGET_BOUNDARY,
        DISABLE_TARGET_SMOOTHING, 
        EMERGENCY_LIMELIGHT_RAW //TODO when have result functions plain limelight
    }

    private enum ManualBackupMode{
        NONE,
        FULL_MANUAL_UNTIL_LIMELIGHT,
        MANUAL_OFFSETS,
        FULL_MANUAL_TARGET_POSITION
    }

    private enum ErrorMode{
        NORMAL,
        LIMELIGHT_TARGET_NOT_SEEN,
        UNEXPECTED_DIFFERENCE_BETWEEN_ODOMETRY_AND_LIMELIGHT,
        LIMELIGHT_CANT_HOLD_TARGET,
        SUSPECTED_BUMP,
        NOT_REACHING_TARGET, //TODO
    }

    public void test(boolean aiming){
        //read dashboard information
        mode = TrackingMode.values()[(int)SmartDashboard.getNumber("tracking mode", mode.ordinal())];
        backup = ManualBackupMode.values()[(int)SmartDashboard.getNumber("backup mode", backup.ordinal())];
        error = ErrorMode.values()[(int)SmartDashboard.getNumber("error mode", error.ordinal())];

        update(aiming);
        //write dashboard information
        SmartDashboard.putNumber("odometry x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odometry y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("odometry heading", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("pigeon heading", pigeon.getHeading());
        SmartDashboard.putNumber("odometry offset x", odometryOffset.getX());
        SmartDashboard.putNumber("odometry offset y", odometryOffset.getY());
        SmartDashboard.putNumber("tracking mode", mode.ordinal());
        SmartDashboard.putNumber("backup mode", backup.ordinal());
        SmartDashboard.putNumber("error mode", error.ordinal());
        SmartDashboard.putNumber("turret position", turret.getAngle().getDegrees());
        SmartDashboard.putNumber("limelight raw x", limelight.getTargetOffsetX());
        SmartDashboard.putNumber("limelight raw y", limelight.getTargetOffsetY());
        SmartDashboard.putNumberArray("limelight target area graph", limelightTargetAreaBuffer);
        SmartDashboard.putNumber("limelight frta", fieldRelativeTargetAngleFromLimelight().getDegrees());
        // SmartDashboard.putBoolean("target in bounds", targetInBounds());
        // SmartDashboard.putNumber("filtered target x", filterLimelightTarget().x());
        // SmartDashboard.putNumber("filtered target y", filterLimelightTarget().y());
        // SmartDashboard.putNumber("manual offset x", manualOffset.getX());
        // SmartDashboard.putNumber("manual offset y", manualOffset.getY());
        // SmartDashboard.putNumber("final target offset", getTargetAngle().getDegrees());
        // SmartDashboard.putNumber("final target distance", getTargetDistance());
    }
}