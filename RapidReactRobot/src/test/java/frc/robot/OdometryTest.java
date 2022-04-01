package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Test;

import static frc.robot.Constants.FieldConstants.kCenterOfHubPose;

public class OdometryTest {

    @Test
    public void testAimToHub() {
        Pose2d hubPose = kCenterOfHubPose;

        Pose2d robotPose = new Pose2d(8.85, 6.94, Rotation2d.fromDegrees(0).minus(Rotation2d.fromDegrees(180)));

        Pose2d targetRobotRelative = kCenterOfHubPose.relativeTo(robotPose);

        Rotation2d theta = new Rotation2d(Math.atan2(targetRobotRelative.getY(), targetRobotRelative.getX()));

        System.out.println(theta);
    }

}
