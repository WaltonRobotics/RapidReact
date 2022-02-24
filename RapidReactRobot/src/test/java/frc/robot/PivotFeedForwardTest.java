package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Assert;
import org.junit.Test;

public class PivotFeedForwardTest {

    // Pivot arm angle from negative x-axis
    public Rotation2d getPivotAngleFromHorizontal(Rotation2d angleFromVertical) {
        return Rotation2d.fromDegrees(90.0).minus(angleFromVertical);
    }

    // Robot pitch angle is CCW positive (in-phase with climber)
    public double getCalculatedFeedForward(Rotation2d robotPitchAngle, Rotation2d angleFromVertical, double maxGravityFeedForward) {
        Rotation2d robotAngleFromGlobalHorizontal = getPivotAngleFromHorizontal(angleFromVertical).minus(robotPitchAngle);

        // If pivot arm has rotated CW past the robot vertical
        if (robotAngleFromGlobalHorizontal.getDegrees() > 90.0) {
            robotAngleFromGlobalHorizontal = Rotation2d.fromDegrees(180.0).minus(robotAngleFromGlobalHorizontal);

            double cosineScalar = Math.abs(robotAngleFromGlobalHorizontal.getCos());
            return maxGravityFeedForward * cosineScalar;
        }

        double cosineScalar = Math.abs(robotAngleFromGlobalHorizontal.getCos());
        return maxGravityFeedForward * -cosineScalar;
    }

    @Test
    public void testFeedForward() {
        double ff1 = getCalculatedFeedForward(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(90.0), 0.07);

        System.out.println(ff1);
        Assert.assertEquals(-0.07, ff1,1.0e-5);

        double ff2 = getCalculatedFeedForward(Rotation2d.fromDegrees(45.0), Rotation2d.fromDegrees(45.0), 0.07);

        System.out.println(ff2);
        Assert.assertEquals(ff1, ff2, 1.0e-5);

        double ff3 = getCalculatedFeedForward(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(45.0), 0.07);

        System.out.println(ff3);
        Assert.assertEquals(-0.049497, ff3, 1.0e-5);

        double ff4 = getCalculatedFeedForward(Rotation2d.fromDegrees(-40.0), Rotation2d.fromDegrees(-5.0), 0.07);

        System.out.println(ff4);
        Assert.assertEquals(0.049497, ff4, 1.0e-5);
    }

}
