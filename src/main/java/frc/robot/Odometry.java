package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.lib.Limelight;
import frc.lib.debloating.Pigeon;
import frc.robot.subsystems.DriveTrain;

public class Odometry {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;
    private Pigeon pigeon;
    private DriveTrain driveTrain;
    private Limelight camera;

    public Odometry (DriveTrain driveTrain, Pigeon pigeon, Limelight camera){
        this.driveTrain = driveTrain;
        this.pigeon = pigeon;
        this.camera = camera;

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    public void update() {
        pigeon.update(getHeading());
        odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.lEncoderPosition(), driveTrain.rEncoderPosition());
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * reset the odometry of the robot by giving it a new pose that it will treat as it's current location.
     * @param pose the new pose where the robot is
     */
    public void resetOdometry(Pose2d pose){
        driveTrain.resetEncoders();
        odometry.resetPosition(pose,Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * get the current location of the robot
     * @return the current pose in meters
     */
    public Pose2d getCurrentPose(){
        return odometry.getPoseMeters();
    }

    //pigeon:
    /**
     * zero the robots heading.
     */
    public void zeroHeading(){
        pigeon.zeroHeading();
    }
    /**
     * get the robots yaw from the pigeon.
     * @return the yaw in degrees
     */
    public double getYaw(){
        return pigeon.getYaw();
    }
    /**
     * get the robots pitch from the pigeon.
     * @return the pitch in degrees
     */
    public double getPitch(){
        return pigeon.getPitch();
    }
    /**
     * get the robots roll from the pigeon.
     * @return the roll in degrees
     */
    public double getRoll(){
        return pigeon.getRoll();
    }
    /**
     * get the fused accelerometer and magnetometer heading from the pigeon
     * @return the heading in degrees
     */
    public double getHeading(){
        return pigeon.getHeading();
    }
}
