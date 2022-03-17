package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.Util;

public class PointTracker {
    private double[] r, p, x, y;
    private final int keep;

    /**
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

    public PointTracker getFuture(){
        Lookaheader lx = new Lookaheader(x[0]);
        Lookaheader ly = new Lookaheader(y[0]);
        for (int i = 0; i < keep; i++){
            lx.add(x[i]);
            ly.add(y[i]);
        }
        double[] newx = Util.shiftLeft(x, lx.compute(10, keep));
        double[] newy = Util.shiftLeft(y, ly.compute(10, keep));
        PointTracker result = new PointTracker(keep);
        for (int i = 0; i < keep; i++){
            result.xy(newx[i], newy[i]);
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

    public static void main(String args[]){
        PointTracker test = new PointTracker(2);
        test.pr(-Rotation2d.fromDegrees(90).getRadians(),1);
        System.out.println(test.x());
        System.out.println(test.y());
    }
}
