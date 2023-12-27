package frc.lib.math;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.Util;

public class PointTracker {
    private double[] r, p, x, y;
    private final int keep;

    /**
     * a function for tracking a point over time, predicting its future value,
     * smoothing it's changes, and converting between cartesian and polar coordinate values
     * @param keep the number of history values to keep for processing
     */
    public PointTracker(int keep){
        this.keep = keep;
        r = new double[keep];
        p = new double[keep];
        x = new double[keep];
        y = new double[keep];
        for(int i = 0; i < keep; i++){
            r[i] = 0;
            p[i] = 0;
            x[i] = 0;
            y[i] = 0;
        }
    }

    // public PointTracker(double x, double y){
    //     this.x = x;
    //     this.y = y;
    //     computePolar();
    // }

    public double p(){
        return p[p.length - 1];
    }

    public double r(){
        return r[r.length - 1];
    }

    public double x(){
        return x[x.length - 1];
    }

    public double y(){
        return y[y.length - 1];
    }

    public PointTracker pr(double p, double r){
        this.p = Util.shiftLeft(this.p, p);
        this.r = Util.shiftLeft(this.r, r);
        computeCartesian();
        return this;
    }

    public PointTracker xy(double x, double y){
        this.x = Util.shiftLeft(this.x, x);
        this.y = Util.shiftLeft(this.y, y);
        computePolar();
        return this;
    }

    /**
     * set the point's current coordinates with a Translation2d
     * @param translation
     * @return this PointTracker
     */
    public PointTracker setTranslation(Translation2d translation){
        xy(translation.getX(), translation.getY());
        return this;
    }

    /**
     * predict the future location of the point with fancy maths
     * @param steps number of steps to the future to compute
     * @return a new PointTracker, with the new prediction
     */
    public PointTracker getFuture(int steps){
        double xPrediction = Util.last(Sequencer.predict(Sequencer.compute(x), steps), 0);
        double yPrediction = Util.last(Sequencer.predict(Sequencer.compute(y), steps), 0);
        double[] newx = Util.shiftLeft(x, xPrediction);
        double[] newy = Util.shiftLeft(y, yPrediction);
        PointTracker result = new PointTracker(keep);
        for (int i = 0; i < keep; i++){
            result.xy(newx[i], newy[i]);
        }
        return result;
    }

    /**
     * get the future location of the point based on the rotation and length (p and r),
     * rather than the x and y value
     * @param steps nuber of steps to the future to compute
     * @return a new PointTracker with the new prediction
     */
    public PointTracker getPolarFuture(int steps){
        double pPrediction = Util.last(Sequencer.predict(Sequencer.compute(p), steps), 0);
        double rPrediction = Util.last(Sequencer.predict(Sequencer.compute(r), steps), 0);
        double[] newx = Util.shiftLeft(p, pPrediction);
        double[] newy = Util.shiftLeft(r, rPrediction);
        PointTracker result = new PointTracker(keep);
        for (int i = 0; i < keep; i++){
            result.pr(newx[i], newy[i]);
        }
        return result;
    }

    /**
     * apply a smoothing function to the point.
     * @param factor how much smoothing to apply, higher = more, 1 = no smoothing
     * @param ceiling ceiling on how much jerk to allow. zero for no jerk.
     * @return
     */
    public PointTracker smooth(double factor, double ceiling){
        double[] xComputed = Sequencer.compute(x);
        double[] yComputed = Sequencer.compute(y);
        for(int i = 1; i < keep; i++){
            xComputed[i] /= factor;
            yComputed[i] /= factor;
        }
        for(int i = 3; i < keep; i++){
            xComputed[i] = Util.limit(xComputed[i], ceiling);
            yComputed[i] = Util.limit(yComputed[i], ceiling);
            
        }
        double[] newx = Sequencer.predict(xComputed, keep);
        double[] newy = Sequencer.predict(yComputed, keep);
        PointTracker result = new PointTracker(keep);
        for (int i = 0; i < keep; i++){
            result.xy(newx[i], newy[i]);
        }
        return result;
    }

    /**
     * apply a smoothing function to the point based on the rotation and length (p and r),
     * rather than the x and y value
     * @param factor how much smoothing to apply, higher = more, 1 = no smoothing
     * @param ceiling ceiling on how much jerk to allow. zero for no jerk.
     * @return
     */
    public PointTracker polarSmooth(double factor, double ceiling){
        double[] pComputed = Sequencer.compute(p);
        double[] rComputed = Sequencer.compute(r);
        for(int i = 1; i < keep; i++){
            pComputed[i] /= factor;
            rComputed[i] /= factor;
        }
        for(int i = 3; i < keep; i++){
            pComputed[i] = Util.limit(pComputed[i], ceiling);
            rComputed[i] = Util.limit(rComputed[i], ceiling);
            
        }
        double[] newx = Sequencer.predict(pComputed, keep);
        double[] newy = Sequencer.predict(rComputed, keep);
        PointTracker result = new PointTracker(keep);
        for (int i = 0; i < keep; i++){
            result.pr(newx[i], newy[i]);
        }
        return result;
    }

    private void computePolar(){
        r = Util.shiftLeft(r, Math.sqrt((x() * x()) + (y() * y())));
        p = Util.shiftLeft(p, Math.atan2(y(), x()));
    }

    private void computeCartesian(){
        x = Util.shiftLeft(x, r() * Math.cos(p()));
        y = Util.shiftLeft(y, r() * Math.sin(p()));
    }

    public String toString(){
        return "PointTracker:\n\tx:" 
                + x() + 
                " y:" + y() + 
                "\n\tp:" + p() + 
                " r:" + r() + 
                "\n\thistory - x:" + Arrays.toString(x) + " y:" + Arrays.toString(y);
    }

    public static PointTracker fromTranslation(Translation2d translation){
        return new PointTracker(1).xy(translation.getX(), translation.getY());
    }

    public static void main(String args[]){
        PointTracker test = new PointTracker(5);
        test.xy(1,2);
        test.xy(2,4);
        test.xy(0,6);
        test.xy(0,2);
        test.xy(5,10);
        test.xy(6, 12);
        System.out.println(test); 
        System.out.println(test.getFuture(1));
        System.out.println(test.smooth(10, 0));
    }

    public Translation2d getTranslation() {
        return new Translation2d(x(), y());
    }
}
