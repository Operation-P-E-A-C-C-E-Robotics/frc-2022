package frc.lib.math;

import java.util.ArrayList;
import java.util.List;
import frc.lib.util.Util;

public class Lookaheader {  
    private ArrayList<Double> points = new ArrayList<>();

    // public Lookaheader(double initialValue){
    //     // points.add(new TimePoint(initialValue, Timer.getFPGATimestamp()));
    //     points.add(initialValue);
    //     times.add(Timer.getFPGATimestamp());
    // }

    public Lookaheader(double initialValue){
        // points.add(new TimePoint(initialValue, Timer.getFPGATimestamp()));
        points.add(initialValue);
    }

    // public void add(double value){
    //     // points.add(new TimePoint(value, Timer.getFPGATimestamp()));
    //     points.add(value);
    //     times.add(Timer.getFPGATimestamp());
    // }

    public void add(double value){
        // points.add(new TimePoint(value, Timer.getFPGATimestamp()));
        points.add(value);
    }

    public Double compute(int steps, int len){
        Sequencer sequencer = new Sequencer();
        List<Double> lookingPoints = Util.subList(points, points.size() - len, points.size() - 1);

        sequencer.setValues(new ArrayList<>(lookingPoints));
        ArrayList<Double> computed = sequencer.compute();
        ArrayList<Double> prediction = sequencer.predict(computed, steps, 1);
        return Util.last(prediction, 0);
    }

    public static void main(String args[]){
        Lookaheader test = new Lookaheader(0);
        test.add(1);
        test.add(2);
        test.add(4);
        test.add(2);
        System.out.println(test.compute(1, 4));
    }

    public static class TimePoint{
        public double val, time;
        public TimePoint(double val, double time){
            this.val = val;
            this.time = time;
        }
    }
}
