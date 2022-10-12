package frc.robot;

import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.UtilMethods;
import org.junit.Test;

import static frc.robot.Constants.FieldConstants.kCenterOfHubPose;

public class OdometryTest {

    private double previousEncDistance = 0;
    private Translation2d position;
    private Translation2d startingPosition;
    private com.team254.lib.geometry.Pose2d estimatedRobotPose = new com.team254.lib.geometry.Pose2d();

    public Rotation2d getAzimuthRotation2d() {
        double azimuthCounts = -5.0 / 8.0;
        double radians = 2.0 * Math.PI * azimuthCounts;
        return new Rotation2d(radians);
    }

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
        Rotation2d angle = getAzimuthRotation2d();
        Rotation2d normalizedAngle = Rotation2d.fromDegrees(UtilMethods.restrictAngle(angle.getDegrees(),
                0, 360));

        return normalizedAngle.rotateBy(robotHeading);
    }

    public synchronized void updatePose(Rotation2d robotHeading){
        double currentEncDistance = 1.0;
        double deltaEncDistance = (currentEncDistance - previousEncDistance);
        Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
        Translation2d deltaPosition = new Translation2d(currentWheelAngle.getCos()*deltaEncDistance,
                currentWheelAngle.getSin()*deltaEncDistance);

        deltaPosition = new Translation2d(deltaPosition.x(),
                deltaPosition.y());
        Translation2d updatedPosition = position.translateBy(deltaPosition);
        com.team254.lib.geometry.Pose2d staticWheelPose = new com.team254.lib.geometry.Pose2d(updatedPosition, new com.team254.lib.geometry.Rotation2d(
                robotHeading.getDegrees()));
        com.team254.lib.geometry.Pose2d robotPose = staticWheelPose.transformBy(com.team254.lib.geometry.Pose2d.fromTranslation(startingPosition).inverse());
        position = updatedPosition;
        estimatedRobotPose = robotPose;
        previousEncDistance = currentEncDistance;
    }

    public synchronized void resetPose(com.team254.lib.geometry.Pose2d robotPose){
        Translation2d modulePosition = robotPose.transformBy(com.team254.lib.geometry.Pose2d.fromTranslation(startingPosition)).getTranslation();
        position = modulePosition;
    }

    public synchronized void zeroSensors(com.team254.lib.geometry.Pose2d robotPose) {
        resetPose(robotPose);
        estimatedRobotPose = robotPose;
        previousEncDistance = 0.0;
    }

    @Test
    public void testAimToHub() {
        Pose2d hubPose = kCenterOfHubPose;

        Pose2d robotPose = new Pose2d(3.14, 1.82, Rotation2d.fromDegrees(32.5));

        Pose2d targetRobotRelative = kCenterOfHubPose.relativeTo(robotPose);

        Rotation2d theta = new Rotation2d(Math.atan2(targetRobotRelative.getY(), targetRobotRelative.getX()));

        System.out.println(theta);
    }

    @Test
    public void testDisplacement() {
        previousEncDistance = 0;

        Translation2d startingPose = new Translation2d(1.0, 1.0);

        position = startingPose;
        this.startingPosition = startingPose;

        zeroSensors(new com.team254.lib.geometry.Pose2d(0, 0,
                com.team254.lib.geometry.Rotation2d.fromDegrees(0.0)));

        updatePose(Rotation2d.fromDegrees(0.0));

        System.out.println(estimatedRobotPose);
    }

}
