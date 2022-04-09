package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Sequencer;
import frc.lib.sensors.Pigeon;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class LiftAndHookNext extends CommandBase{
  Climber climber;
  private Joystick joystick;
  private Joystick driverJoystick;
  private boolean finished = false;

  /*
  0: lift to bottom;
  1: extend arm, slowly lower;
  2: extend lift to just below rung;
  3: wait till swing amount correct;
  4: extend hook beyond rung;
  5: hook rung; use gyro to make sure everything going correctly;
  6: lift a bit
  7: finished
  */
  int stage;

  private final double liftFullyExtendedCounts = 0,
                        liftFullExtendedCoastCounts = 0,
                        liftUnderNextHookCounts = 0,
                        liftSoftDropCounts = 0,
                        liftRetractCoastCounts = 0,
                        nextHookGrabbedCounts = 0,
                        finalLiftCounts = 0;

    private final double coastSpeed = 0.2;
    private Pigeon pigeon;
    private double[] pigeonHistory = new double[3];

    private double angleLimit = 0,
                    angleVelocityLimit = 0,
                    angleAccelerationLimit = 0;
    
    private double grabAngleLimit = 0,
                    grabAngleVelocityLimit = 0,
                    grabAngleAccelerationLimit = 0;

  public LiftAndHookNext(Climber climber, Joystick joystick, Pigeon pigeon, RobotContainer container) {
    this.climber = climber;
    this.joystick = joystick;
    this.pigeon = pigeon;
    stage = 0;
    pigeonHistory = Util.zeros(pigeonHistory);
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  @Override
  public void initialize(){
    stage = 0;
    pigeonHistory = Util.zeros(pigeonHistory);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Util.shiftLeft(pigeonHistory, pigeon.getYaw()); //TODO get right (yaw, pitch, roll)
    switch(stage){
        case 0:
            //lift to bottom
            climber.setLiftPercent(-1);
            if(climber.getLiftPosition() < liftRetractCoastCounts) climber.setLiftPercent(-coastSpeed);
            if(climber.liftFullyRetracted()) stage = 1;
            break;
        case 1:
            climber.armsOut();
            climber.setLiftPercent(0.1);
            if(climber.getLiftPosition() > liftSoftDropCounts) stage = 2;
            break;
        case 2:
            climber.setLiftPercent(1);
            if(climber.getLiftPosition() > liftUnderNextHookCounts) stage = 3;
            break;
        case 3:
            climber.setLiftPercent(0);
            double[] computed = Sequencer.compute(pigeonHistory);
            if(computed[0] < angleLimit         &&
               computed[1] < angleVelocityLimit &&
               computed[2] < angleAccelerationLimit) stage = 4; //TODO signs might need to be swapped
            break;
        case 4:
            climber.setLiftPercent(1);
            if(climber.getLiftPosition() > liftFullExtendedCoastCounts) climber.setLiftPercent(coastSpeed);
            if(climber.getLiftPosition() > liftFullyExtendedCounts) stage = 5;
            break;
        case 5:
            double[] computed2 = Sequencer.compute(pigeonHistory);
            //TODO check sign
            if(computed2[0] > grabAngleLimit                       &&
                Util.inRange(computed2[1], grabAngleVelocityLimit) &&
                Util.inRange(computed2[2], grabAngleAccelerationLimit)){
                    climber.setLiftPercent(-1);
            } else {
                climber.setLiftPercent(0);
            }
            if(climber.getLiftPosition() < nextHookGrabbedCounts) stage = 6;
            break;
        case 6:
            climber.setLiftPercent(-1);
            if(climber.getLiftPosition() < finalLiftCounts) stage = 7;
            break;
        case 7:
            climber.setLiftPercent(0);
            finished = true;
            break;
        default:
            System.out.println("ERROR 'LiftAndHook' command switch hit default case. Which it shouldnt. So it will be terminated.");
            climber.setLiftPercent(0);
            finished = true;
            break;
    }
  }

  @Override
  public boolean isFinished(){
      return finished;
  }

  @Override
  public void end(boolean i){
      climber.setLiftPercent(0);
  }
}
