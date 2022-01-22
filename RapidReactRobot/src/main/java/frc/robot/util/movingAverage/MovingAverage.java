package frc.robot.util.movingAverage;

public interface MovingAverage {

    void clear();

    void addData(double num);

    double getMean();

    int getNumValues();

}