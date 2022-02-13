package frc.robot.util;

public interface MovingAverage {

   void addData(double num);

    double getMean();

    int getNumValues();

}
