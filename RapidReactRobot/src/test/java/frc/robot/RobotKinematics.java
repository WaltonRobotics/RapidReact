package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain;
import org.junit.Assert;
import org.junit.Test;

import static frc.robot.Constants.PathFollowing.kPathLookaheadTime;
import static frc.robot.Paths.NewFiveBallRoutine.fiveBall1;
import static frc.robot.Paths.NewFiveBallRoutine.fiveBall2;
import static frc.robot.Paths.TwoBallThrowRoutine.twoBall;
import static frc.robot.RobotContainer.godSubsystem;

public class RobotKinematics {

    @Test
    public void testMaxOmega() {
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

    @Test
    public void testInitialAngle() {
        Drivetrain drivetrain = godSubsystem.getDrivetrain();

        HolonomicDriveController controller = new HolonomicDriveController(
                drivetrain.getConfig().getXController(),
                drivetrain.getConfig().getYController(),
                drivetrain.getConfig().getThetaController());

        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState)fiveBall1.sample(kPathLookaheadTime);

        ChassisSpeeds speeds = controller.calculate(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(),
                state.holonomicRotation), state, state.holonomicRotation);

        Rotation2d initialAngle = godSubsystem.getDrivetrain().getSwerveDriveKinematics().toSwerveModuleStates(speeds)[0].angle;

        System.out.println(initialAngle);

        Assert.assertEquals(-90, initialAngle.getDegrees(), 0.5);
    }

    @Test
    public void testForwardKinematics() {
        ChassisSpeeds chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        0,
                        Rotation2d.fromDegrees(0));

        var swerveModuleStates =
                godSubsystem.getDrivetrain().getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                godSubsystem.getDrivetrain().getConfig().getMaxSpeedMetersPerSecond());

        System.out.println(swerveModuleStates[0].angle);
        System.out.println(swerveModuleStates[1].angle);
        System.out.println(swerveModuleStates[2].angle);
        System.out.println(swerveModuleStates[3].angle);
    }

}
