package frc.robot;

import org.junit.Test;

public class ExtensionBaselineLimits {

    private double getBaselineTargetNU(double inches, double spoolDiameter) {
        return Math.round(inches * (1 / (Math.PI * spoolDiameter)) * 36 * 2048);
    }

    @Test
    public void calculateTargetsAndLimits() {
        final double spoolDiameter = 1.25;
        final double tolerance = getBaselineTargetNU(0.1, spoolDiameter);

        System.out.println("Tolerance: " + tolerance);

        System.out.println("Limits:");

        double stowedRev = getBaselineTargetNU(0, spoolDiameter) - tolerance;
        double stowedFwd = getBaselineTargetNU(0, spoolDiameter) + tolerance;
        System.out.println(stowedRev + " " + stowedFwd);

        double fullRomRev = getBaselineTargetNU(0, spoolDiameter) - tolerance;
        double fullRomFwd = getBaselineTargetNU(26.0 - 1, spoolDiameter) + tolerance;
        System.out.println(fullRomRev + " " + fullRomFwd);

        double midBarRev = getBaselineTargetNU(11.0 - 1, spoolDiameter) - tolerance;
        double midBarFwd = getBaselineTargetNU(11.0 - 1, spoolDiameter) + tolerance;
        System.out.println(midBarRev + " " + midBarFwd);

        double highBarRev = getBaselineTargetNU(13.50 - 1, spoolDiameter) - tolerance;
        double highBarFwd = getBaselineTargetNU(13.50 - 1, spoolDiameter) + tolerance;
        System.out.println(highBarRev + " " + highBarFwd);

        System.out.println("Targets:");
        System.out.println(getBaselineTargetNU(0, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(21.467 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(11.0 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(3.0 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(25 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(13.50 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(3.0 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(26.0 - 1, spoolDiameter) + " " + tolerance);
        System.out.println(getBaselineTargetNU(11.6 - 1, spoolDiameter) + " " + tolerance);
    }

}
