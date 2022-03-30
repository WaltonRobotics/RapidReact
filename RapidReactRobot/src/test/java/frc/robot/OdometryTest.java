package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Test;

import static frc.robot.Constants.FieldConstants.kCenterOfHubPose;

public class OdometryTest {

    @Test
    public void testAimToHub() {
        Pose2d hubPose = kCenterOfHubPose;
        Pose2d robotPose = new Pose2d(7.65, 1.75, Rotation2d.fromDegrees(0.0));
    }

}
