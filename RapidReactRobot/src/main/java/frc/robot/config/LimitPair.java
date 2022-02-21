package frc.robot.config;

public class LimitPair {

    private final double reverseSoftLimitThreshold;
    private final double forwardsSoftLimitThreshold;

    public LimitPair(double reverseSoftLimitThreshold, double forwardsSoftLimitThreshold) {
        this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
        this.forwardsSoftLimitThreshold = forwardsSoftLimitThreshold;
    }

    public double getReverseSoftLimitThreshold() {
        return reverseSoftLimitThreshold;
    }

    public double getForwardsSoftLimitThreshold() {
        return forwardsSoftLimitThreshold;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null) return false;
        if (!(obj instanceof LimitPair)) return false;

        LimitPair pair = (LimitPair) obj;
        return reverseSoftLimitThreshold == pair.reverseSoftLimitThreshold
                && forwardsSoftLimitThreshold == pair.forwardsSoftLimitThreshold;
    }

    @Override
    public String toString() {
        return "LimitPair{" +
                "reverseSoftLimitThreshold=" + reverseSoftLimitThreshold +
                ", forwardsSoftLimitThreshold=" + forwardsSoftLimitThreshold +
                '}';
    }

}
