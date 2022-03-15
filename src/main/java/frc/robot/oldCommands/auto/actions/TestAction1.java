package frc.robot.oldCommands.auto.actions;

import frc.lib.auto.Action;
import frc.robot.subsystems.BallHandler;

public class TestAction1 extends Action{
    private BallHandler intake;

    public TestAction1(BallHandler intake){
        this.intake = intake;
    }

    @Override
    public void init() {
        intake.armsDown();
        intake.setIntake(1);
        intake.setTraversal(1);        
    }

    @Override
    public void execute() {
        
    }
    
}
