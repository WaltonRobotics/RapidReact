package frc.robot.config;

public class LimitPair {

    private final double forwardsSoftLimitThreshold;
    private final double reverseSoftLimitThreshold;

    public LimitPair(double forwardsSoftLimitThreshold, double reverseSoftLimitThreshold) {
        this.forwardsSoftLimitThreshold = forwardsSoftLimitThreshold;
        this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
    }

    public double getForwardsSoftLimitThreshold() {
        return forwardsSoftLimitThreshold;
    }

    public double getReverseSoftLimitThreshold() {
        return reverseSoftLimitThreshold;
    }

    @Override
    public String toString() {
        return "LimitPair{" +
                "forwardsSoftLimitThreshold=" + forwardsSoftLimitThreshold +
                ", reverseSoftLimitThreshold=" + reverseSoftLimitThreshold +
                '}';
    }

}
