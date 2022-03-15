package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.lib.Limelight;
import frc.lib.math.PointTracker;
import frc.robot.subsystems.Turret;

public class TargetTracker {
    private Limelight limelight;
    private DifferentialDriveOdometry odometry;
    private Pose2d odo_offset;
    private Pose2d ll_offset;
    private Turret turret;
    private PointTracker target;

    public TargetTracker(Limelight limelight, DifferentialDriveOdometry odometry, Turret turret, Pose2d odometryOffset, Pose2d limelightOffset){
        this.limelight = limelight;
        this.odometry = odometry;
        this.turret = turret;
        this.odo_offset = odometryOffset;
        this.ll_offset = limelightOffset;
        target = new PointTracker(3);
    }

    public Pose2d getRoughPoseLimelight(){
        PointTracker pose = new PointTracker(1);
        double absoluteTargetRotation = (turret.getPosition() * 2 * Math.PI) + ((limelight.getTargetOffsetX() / 360) * 2 * Math.PI);
        pose.pr(absoluteTargetRotation, limelight.getTargetDistance());
        return new Pose2d(pose.x(), pose.y(), new Rotation2d(absoluteTargetRotation)).relativeTo(ll_offset);
    }

    public Pose2d getRoughPoseOdometry(){
        return odometry.getPoseMeters().relativeTo(odo_offset);
    }

    public void update(){
        Pose2d odoPose = getRoughPoseOdometry();

        if(limelight.hasTarget() == 1){
            Pose2d llPose = getRoughPoseLimelight();
            
            //update odometry offset with limelight
            Pose2d difference = odoPose.relativeTo(llPose);
            difference = new Pose2d(
                distanceFalloff(difference.getX(), 1, 1), //make large variations take longer to equalize 
                distanceFalloff(difference.getY(), 1, 1), 
                Rotation2d.fromDegrees(
                    distanceFalloff(difference.getRotation().getDegrees(), 7, 1)
                )
            );
            odo_offset.relativeTo(difference); //TODO check if any of these backwards
        }

        target.xy(odoPose.getX(), odoPose.getY());
    }

    public double getTargetDistance(){
        return target.getFuture().p();
    }

    public double getTargetAngle(){
        return target.getFuture().r();
    }
    
    private double distanceFalloff(double value, double amount, double multiplier){
        // boolean neg = value < 0;
        double res = (value * multiplier/Math.sqrt(amount * (Math.abs(value) + 1)));
        return res;
    }
}
