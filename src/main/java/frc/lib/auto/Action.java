package frc.lib.auto;

public abstract class Action {
    private boolean running = false;

    public abstract void init();
    public abstract void execute();

    public boolean finished(boolean timeUp){ return timeUp; }
    public double duration() { return Double.NaN; }

    public void run(){
        if (!running) init();
        else execute();
    }

    public boolean running(){
        return running;
    }
}
