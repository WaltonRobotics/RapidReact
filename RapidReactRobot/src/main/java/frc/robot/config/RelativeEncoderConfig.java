package frc.robot.config;

public interface RelativeEncoderConfig {

    double getDistancePerPulse();

    int getChannelA();
    int getChannelB();

    boolean isInverted();

}
