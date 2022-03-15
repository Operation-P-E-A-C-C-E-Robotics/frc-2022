package frc.lib.math;

import java.util.ArrayList;
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
    public static void main(String args[]){
        Sequencer test = new Sequencer();
        test.setValues(new ArrayList<>(List.of(1.0,4.0,9.0,16.0)));
        ArrayList<Double> computed = test.compute();
        System.out.println(computed);
        System.out.println(test.predict(computed, 1, 1));
    }
}
