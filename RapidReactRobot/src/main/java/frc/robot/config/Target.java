package frc.robot.config;

import frc.robot.util.UtilMethods;

public class Target {

    private final double target;
    private final double tolerance;

    public Target(double target, double tolerance) {
        this.target = target;
        this.tolerance = tolerance;
    }

    public Target(double target) {
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
        return UtilMethods.isWithinTolerance(measured, target, tolerance);
    }

    @Override
    public String toString() {
        return "Target{" +
                "target=" + target +
                ", tolerance=" + tolerance +
                '}';
    }
}
