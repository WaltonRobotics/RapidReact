package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Superstructure;

import static frc.robot.Constants.FieldConstants.kCenterOfHubPose;
import static frc.robot.RobotContainer.godSubsystem;

public class LiveDashboardHelper {

    public static Pose2d reflectPose(Pose2d oldPose) {
        Translation2d oldTranslation = oldPose.getTranslation();

        // Reflect robot translation over origin if on red
        double dx = kCenterOfHubPose.getX() - oldTranslation.getX();
        double dy = kCenterOfHubPose.getY() - oldTranslation.getY();

        Translation2d newTranslation = new Translation2d(kCenterOfHubPose.getX() + dx,
                kCenterOfHubPose.getY() + dy);

        // Flip the robot heading since the robot 0 is now facing in the direction of the blue alliance

        return new Pose2d(newTranslation,
                oldPose.getRotation().minus(Rotation2d.fromDegrees(180)));
    }

    public static void putRobotData(Pose2d currentPose) {
        LiveDashboardTable.getInstance().setRobotX(Units.metersToFeet(currentPose.getTranslation().getX()));
        LiveDashboardTable.getInstance().setRobotY(Units.metersToFeet(currentPose.getTranslation().getY()));
        LiveDashboardTable.getInstance().setRobotHeading(currentPose.getRotation().getRadians());
    }

    public static void putTrajectoryData(Pose2d trajectoryPose) {
        if (godSubsystem.getInferredAllianceColor() == Superstructure.AllianceColor.RED) {
            trajectoryPose = reflectPose(trajectoryPose);
        }

        LiveDashboardTable.getInstance().setPathX(Units.metersToFeet(trajectoryPose.getTranslation().getX()));
        LiveDashboardTable.getInstance().setPathY(Units.metersToFeet(trajectoryPose.getTranslation().getY()));
        LiveDashboardTable.getInstance().setPathHeading(trajectoryPose.getRotation().getRadians());
    }

}
