package frc.robot.util;// copied from GameChangers code

import java.util.LinkedList;
import java.util.Queue;

public class FindAverage implements frc.robot.utils.movingAverage.MovingAverage {

    // queue used to store list so that we get the average
    private final Queue<Double> dataset = new LinkedList<>();
    private final int period;
    private double sum;

    // constructor to initialize period
    public FindAverage(int period) {
        this.period = period;

        clear();
    }

    @Override
    public void clear() {
        dataset.clear();
        sum = 0;
    }

    /* function to add new data in the
       list and update the sum so that
       we get the new mean */
    @Override
    public void addData(double num) {
        sum += num;
        dataset.add(num);

        /* Updating size so that length
           of data set should be equal
           to period as a normal mean has */
        if (dataset.size() > period) {
            sum -= dataset.remove();
        }
    }

    // function to calculate mean
    @Override
    public double getMean() {
        return sum / period;
    }

    @Override
    public int getNumValues() {
        return dataset.size();
    }

}
