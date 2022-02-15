package frc.robot.config;

public class Target {

    private final double target;
    private final double tolerance;

    public Target(int target, int tolerance) {
        this.target = target;
        this.tolerance = tolerance;
    }

    public Target(int target) {
        this(target, 0);
    }

    public double getTarget() {
        return target;
    }

    public double getTolerance() {
        return tolerance;
    }

    public boolean isWithinTolerance(double measured) {
        return isWithinTolerance(measured, tolerance);
    }

    public boolean isWithinTolerance(double measured, double tolerance) {
        return Math.abs(measured - target) <= tolerance;
    }

    @Override
    public String toString() {
        return "Target{" +
                "target=" + target +
                ", tolerance=" + tolerance +
                '}';
    }
}
