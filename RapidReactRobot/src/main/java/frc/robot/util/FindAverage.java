package frc.robot.util;// copied from GameChangers code

import java.util.LinkedList;
import java.util.Queue;

public class FindAverage implements frc.robot.utils.movingAverage.MovingAverage {

    // queue used to store list so that we get the average
    private final Queue<Double> dataset = new LinkedList<>();
    private double sum;



    /* function to add new data in the
       list and update the sum so that
       we get the new mean */
    @Override
    public void addData(double num) {
        sum += num;
        dataset.add(num);
    }

    @Override
    public int getNumValues() {
        return dataset.size();
    }

    // function to calculate mean
    @Override
    public double getMean() {
        return sum / getNumValues();
    }


}
