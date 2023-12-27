package frc.robot;

import org.w3c.dom.Text;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Joystick;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Pigeon;
import frc.lib.util.TargetTracker;
import frc.lib.util.TargetTracker_old;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.OldTurret;
import frc.robot.subsystems.Turret;

public class Odometry {
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6);
    private final DifferentialDriveOdometry odometry;
    private final Pigeon pigeon;
    // private TargetTracker_old target;
    TargetTracker target;
    private final DriveTrain driveTrain;
    private Limelight camera;
    private Joystick joystick;

    public Odometry (DriveTrain driveTrain, Turret turret, Pigeon pigeon, Limelight camera, Joystick operatorJoystick){
        this.driveTrain = driveTrain;
        this.pigeon = pigeon;
        this.joystick = operatorJoystick;
        this.camera = camera;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        target = new TargetTracker(camera, turret, pigeon, odometry, new Pose2d(), new Translation2d(), operatorJoystick);
    }

    public TargetTracker getTarget(){
        return target;
    }

    public void update() {
        // target.update();
        camera.update();
        pigeon.update(getHeading());
        odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.lEncoderPosition(), driveTrain.rEncoderPosition());
        target.test(joystick.getRawButton(5));
        // SmartDashboard.putNumber("robot y", getCurrentPose().getY());
        // SmartDashboard.putNumber("robot heading", getCurrentPose().getRotation().getDegrees());
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
        return -pigeon.getHeading();
    }
}
