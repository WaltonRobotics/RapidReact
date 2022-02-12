package frc.robot.config;

public interface EncoderConfig {

    double getDistancePerPulse();

    int getChannelA();

    int getChannelB();

    boolean isInverted();

}
