package frc.robot.util.averages;

public interface MovingAverage {

    void clear();

    void addData(double num);

    double getMean();

    int getNumValues();

}