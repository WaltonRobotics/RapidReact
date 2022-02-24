package frc.robot.config;

import edu.wpi.first.math.controller.PIDController;

public interface SmartMotionConstants {

    PIDController getVelocityPID();

    double getIZone();

    double getFeedforward();

    double getMinOutput();

    double getMaxOutput();

    double getMaxVelocity();

    double getMinOutputVelocity();

    double getMaxAccel();

    double getAllowedClosedLoopError();

}
