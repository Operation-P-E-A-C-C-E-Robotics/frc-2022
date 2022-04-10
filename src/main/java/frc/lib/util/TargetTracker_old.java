package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.PointTracker;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Turret;

public class TargetTracker_old {
    private Limelight limelight;
    private Pigeon pigeon;
    private DifferentialDriveOdometry odometry;

    private Turret turret;
    
    private Pose2d pose;
    private PointTracker targetRelativeToRobot;
    private final Translation2d targetPosition;

    public TargetTracker_old(Limelight limelight, Pigeon pigeon, DifferentialDriveOdometry odometry, Turret turret, Translation2d targetPosition){
        this.limelight = limelight;
        this.odometry = odometry;
        this.turret = turret;
        this.targetPosition = targetPosition;
        this.pigeon = pigeon;
        targetRelativeToRobot = new PointTracker(3);
    }

    /**
     * use the limelight to get the target's point relative to the robot
     * @return the point, with x = forward, y = left
     */
    public PointTracker getTargetRelativeToRobotLimelight(){
        double dist = limelight.getTargetDistance();
        double rot = - (turret.getPosition() * 2 * Math.PI) - ((limelight.getTargetOffsetX() / 360) * 2 * Math.PI) + ((pigeon.getHeading() / 360) * 2 * Math.PI);
        PointTracker point = new PointTracker(1).pr(rot, dist); //TODO check rotation negative
        return point;
    }

    /**
     * get pose of the robot from the limelight, relative to the target position
     * @return
     */
    public Pose2d getRobotPoseLimelight(){
        PointTracker robot = getTargetRelativeToRobotLimelight();
        robot.xy(robot.x() + targetPosition.getX(), robot.y() + targetPosition.getY());
        // double rot = //- (turret.getPosition() * 2 * Math.PI) - ((limelight.getTargetOffsetX() / 360) * 2 * Math.PI);
        return new Pose2d(robot.x(), robot.y(), Rotation2d.fromDegrees(pigeon.getHeading()));//new Rotation2d(robot.p() - rot)); //TODO check rotation math
    }

    public void update(){
        Pose2d odoPose = odometry.getPoseMeters();
        if(limelight.hasTarget() == 1){
            Pose2d llPose = getRobotPoseLimelight();
            SmartDashboard.putNumber("ll pose x", llPose.getX());
            SmartDashboard.putNumber("ll pose y", llPose.getY());
            SmartDashboard.putNumber("ll pose rot", llPose.getRotation().getDegrees());
            Pose2d odoReset = new Pose2d(
                odoPose.getX() - llPose.getX(),
                odoPose.getY() - llPose.getY(),
                new Rotation2d(odoPose.getRotation().getRadians() - llPose.getRotation().getRadians())
            );
            odometry.resetPosition(odoReset, Rotation2d.fromDegrees(pigeon.getHeading()));
        }
        targetRelativeToRobot.xy(odoPose.getX() + targetPosition.getX(), odoPose.getY() + targetPosition.getY());
    }

    public double getTargetDistance(){
        return targetRelativeToRobot.p();
    }

    public double getTargetAngle(){
        return targetRelativeToRobot.r();
    }
    
    private double distanceFalloff(double value, double amount, double multiplier){
        // boolean neg = value < 0;
        double res = (value * multiplier/Math.sqrt(amount * (Math.abs(value) + 1)));
        return res;
    }
}
