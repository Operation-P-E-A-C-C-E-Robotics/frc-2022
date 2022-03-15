package frc.lib.math;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
public class Sequencer {
    ArrayList<Double> values = new ArrayList<>();
    public Sequencer(){

    }
    public void setValues(ArrayList<Double> values){
        this.values = values;
    }
    public ArrayList<Double> compute(){
        ArrayList<Double> computed = new ArrayList<>();
        ArrayList<Double> working = values;

        computed.add(values.get(0));

        for(int j = 1; j < values.size(); j++){
            ArrayList<Double> newWorking = new ArrayList<>();
            for(int i = 1; i < working.size(); i++){
                double v1 = working.get(i);
                double v2 = working.get(i - 1);
                // boolean isNegative = (Double.compare(v1, 0) <= -0) ^ (Double.compare(v2, 0) <= -0);
                // newWorking.add(isNegative ? -(Math.abs(v1) - Math.abs(v2)) : (Math.abs(v1) - Math.abs(v2)));
                newWorking.add(v1 - v2);
            }
            working = newWorking;
            computed.add(working.get(0));
        }
        return computed;
    }
    public ArrayList<Double> predict(ArrayList<Double> computed, int len, double stepSize){
        ArrayList<Double> working = new ArrayList<>();

        for(int i = 0; i <= len; i++){
            working.add(computed.get(computed.size() - 1) * stepSize);
        }

        for (int i = computed.size() - 2; i >= 0; i--){
            ArrayList<Double> newWorking = new ArrayList<>();
            newWorking.add(computed.get(i));
            for(Double v2 : working){
                Double v1 = newWorking.get(newWorking.size() - 1);
                // boolean isNegative = (Double.compare(v1, 0) < 0) ^ (Double.compare(v2, 0) < 0);
                // newWorking.add(isNegative ? -(Math.abs(v1) + Math.abs(v2)) : (Math.abs(v1) + Math.abs(v2)));
                newWorking.add(v1 + v2);
            }
            working = newWorking;
        }
        return working;
    }
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
        // Sequencer test = new Sequencer();
        // test.setValues(new ArrayList<>(List.of(1.0,4.0,9.0,16.0)));
        // ArrayList<Double> computed = test.compute();
        // System.out.println(computed);
        // System.out.println(test.predict(computed, 1, 1));
        double[] test = {1,1,0};
        double[] res = Sequencer.predict(test, 50);
        for (double i : res) System.out.println(i);
    }
}
