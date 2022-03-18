package frc.lib.math;

import java.util.ArrayList;

public class Sequencer {
    public static double[] predict(double[] computed, int numpredictions){
        double[] current = new double[numpredictions + 1];
        for(int i = 0; i < numpredictions; i++){
            current[i] = computed[computed.length - 1];
        }
        for(int i = computed.length - 2; i >= 0; i--){
            current = stepUp(current, computed[i]);
        }
        return current;
    }

    public static double[] compute(double[] input){
        double[] res = new double[input.length];
        double[] current = input;
        for (int i = 0; i < input.length; i++){
            res[i] = current[0];
            current = stepDown(current);
        }
        return res;
    }

    public static double[] stepUp(double[] in, double start){
        double[] result = new double[in.length + 1];
        result[0] = start;
        for(int i = 0; i < in.length; i++){
            result[i+1] = result[i] + in[i];
        }
        return result;
    }
    
    public static double[] stepDown(double[] in){
        double[] result = new double[in.length - 1];
        for (int i = 1; i < in.length; i++){
            result[i - 1] = in[i] - in[i - 1];
        }
        return result;
    }
    public static void main(String args[]){
        double[] test = {1,4,9,16};
        double[] res = Sequencer.predict(Sequencer.compute(test), 50);
        for (double i : res) System.out.println(i);
    }
}
