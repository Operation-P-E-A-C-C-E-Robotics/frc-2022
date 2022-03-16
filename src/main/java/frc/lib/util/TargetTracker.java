package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Limelight;
import frc.lib.debloating.Pigeon;
import frc.lib.math.PointTracker;
import frc.robot.subsystems.Turret;

public class TargetTracker {
    private Limelight limelight;
    private DifferentialDriveOdometry odometry;
    private Pose2d odo_offset;
    private Pose2d ll_offset;
    private Turret turret;
    private PointTracker target;
    private Pigeon pigeon;

    public TargetTracker(Limelight limelight, Pigeon pigeon, DifferentialDriveOdometry odometry, Turret turret, Pose2d odometryOffset, Pose2d limelightOffset){
        this.limelight = limelight;
        this.odometry = odometry;
        this.turret = turret;
        this.odo_offset = odometryOffset;
        this.ll_offset = limelightOffset;
        this.pigeon = pigeon;
        target = new PointTracker(3);
    }

    // public Pose2d getRoughPoseLimelight(){
    //     PointTracker pose = new PointTracker(1);
    //     double absoluteTargetRotation = -(turret.getPosition() * 2 * Math.PI) + ((limelight.getTargetOffsetX() / 360) * 2 * Math.PI);
    //     pose.pr(absoluteTargetRotation, limelight.getTargetDistance());
    //     return new Pose2d(pose.x(), pose.y(), new Rotation2d(absoluteTargetRotation));//.relativeTo(ll_offset);
    // }

    public Pose2d getRoughPoseOdometry(){
        Pose2d pose = odometry.getPoseMeters().relativeTo(odo_offset);
        pose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getRadians() - pigeon.getHeading()));
        return pose;
        // return odo_offset.relativeTo(odometry.getPoseMeters());
    }

    public void update(){
        Pose2d odoPose = getRoughPoseOdometry();

        // if(limelight.hasTarget() == 1){
        //     Pose2d llPose = getRoughPoseLimelight();
        //     SmartDashboard.putNumber("ll x", llPose.getX());
        //     SmartDashboard.putNumber("ll y", llPose.getY());
        //     SmartDashboard.putNumber("ll rot", llPose.getRotation().getDegrees());
 
        //     SmartDashboard.putNumber("offset x", odoPose.getX());
        //     SmartDashboard.putNumber("offset y", odoPose.getY());
        //     SmartDashboard.putNumber("offset rot", odoPose.getRotation().getDegrees());
        //     //update odometry offset with limelight
        //     // Pose2d difference = new Pose2d(
        //     //     odoPose.getX() - llPose.getX(),
        //     //     odoPose.getY() - llPose.getY(),
        //     //     new Rotation2d(odoPose.getRotation().getRadians() - llPose.getRotation().getRadians());
        //     // );
        //     Pose2d difference = llPose.relativeTo(odoPose);
        //     difference = new Pose2d(
        //         distanceFalloff(difference.getX(), 7, 1), //make large variations take longer to equalize 
        //         distanceFalloff(difference.getY(), 7, 1), 
        //         Rotation2d.fromDegrees(
        //             distanceFalloff(difference.getRotation().getDegrees(), 7, 1)/1000
        //         )
        //     );
        //     odo_offset = difference.relativeTo(odo_offset); //TODO check if any of these backwards
        // }

        target.xy(odoPose.getX(), odoPose.getY());
    }

    public double getTargetDistance(){
        return target.p();
    }

    public double getTargetAngle(){
        return target.r();
    }
    
    private double distanceFalloff(double value, double amount, double multiplier){
        // boolean neg = value < 0;
        double res = (value * multiplier/Math.sqrt(amount * (Math.abs(value) + 1)));
        return res;
    }
}
