package frc.lib.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class TimelineAuto {
    private ArrayList<TimedRunner> actions = new ArrayList<>();
    private double timerOffset = Timer.getFPGATimestamp();

    public void add(TimedRunner action){
        actions.add(action);
    }

    public void start(){
        timerOffset = Timer.getFPGATimestamp();
    }

    public void update(){
        double time = Timer.getFPGATimestamp() - timerOffset;
        for (int i = 0; i < actions.size(); i++){
            TimedRunner action = actions.get(i);
            timerOffset = Timer.getFPGATimestamp() - action.editTimer(time);
        }

        time = Timer.getFPGATimestamp() - timerOffset;
        for (int i = 0; i < actions.size(); i++){
            TimedRunner action = actions.get(i);
            action.execute(time);
        }
    }

    public double getMaxTime(){
        double max = 0;
        for (TimedRunner i : actions){
            max = i.getStopTime() > max ? i.getStopTime() : max;
        }
        return max;
    }
}
