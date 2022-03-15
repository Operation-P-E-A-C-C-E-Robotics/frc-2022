/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oldCommands.auto.paths;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Odometry;
import frc.robot.subsystems.DriveTrain;

/**
 * Testing the PathBase framework. just call start on this to drive the path.
 */
public class OffLinePath extends PathBase {
    /**
     * creates a new trajectory, and then sets it in the PathBase as the one to
     * follow. Call start on this to drive the path.
     * 
     * @param subsystem drive train to pass to PathBase
     * @throws IOException
     */
    public OffLinePath(DriveTrain subsystem, Odometry odometry) throws IOException {
        super(subsystem, odometry);
        //create a new trajectoryP
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)),
        List.of(
            new Translation2d(0.5,0)
        ),
        new Pose2d(1,0, Rotation2d.fromDegrees(0)),
        getTrajectoryConfig()
        );
        //set the trajectory
        setTrajectory(trajectory);
        System.out.println("trajectory ready");
	}
}
