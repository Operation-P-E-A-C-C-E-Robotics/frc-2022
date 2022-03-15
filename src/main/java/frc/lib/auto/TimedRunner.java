package frc.lib.auto;

public class TimedRunner {
    private Action action;
    private double start;
    private double stop;

    private final double padding = 0.1;

    public TimedRunner(Action action, double start, double stop){
        this.action = action;
        this.start = start;
        this.stop = stop;
    }

    public double editTimer(double time){
        if (time < stop) return time;
        if (action.finished(true)) return time;
        return stop - padding;
    }

    public void execute(double time){
        if (!action.finished(time > start && time <=stop)) {
            action.run();
        }
    }

    public double getStopTime(){
        return stop;
    }
}
