package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.Assert;
import org.junit.Test;

public class RobotKinematics {

    @Test
    public void testKinematics() {
        // Bumper-bumper length: 33.489 in
        // Bumper-bumper width: 26.458 in
        final double kDistanceBetweenWheelsWidthWiseMeters =
                Units.inchesToMeters(13.173279);

        final double kDistanceBetweenWheelsLengthWiseMeters =
                Units.inchesToMeters(20.173279);

        final double kMaxSpeedMetersPerSecond = 3.889;
        final double kMaxOmega = (kMaxSpeedMetersPerSecond / Math.hypot(kDistanceBetweenWheelsLengthWiseMeters / 2.0,
                kDistanceBetweenWheelsWidthWiseMeters / 2.0))
                / 2.0;

        System.out.println(kMaxOmega);
        Assert.assertEquals(kMaxOmega, 6.354837311073039, 0.1);
    }

}
