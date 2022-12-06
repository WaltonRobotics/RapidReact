package frc.robot.config;

public interface AbsoluteEncoderConfig {

    double getDistancePerRotation();

    int getChannel();

    boolean isInverted();

}
