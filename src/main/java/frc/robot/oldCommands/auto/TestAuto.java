// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

package frc.robot.oldCommands.auto;

import java.io.IOException;

import frc.lib.auto.Action;
import frc.lib.auto.Auto;
import frc.robot.oldCommands.auto.actions.TestAction1;
import frc.robot.subsystems.BallHandler;


/**
 * A file to test the auto framework.
 */
public class TestAuto{
    public TestAuto(BallHandler intake) {
        Auto auto = new Auto();
        auto.duration(new TestAction1(intake), 5);
    }
}
