package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.WaltSwerveModule;
import frc.robot.util.UtilMethods;
import frc.robot.util.averages.CumulativeAverage;

import java.util.ArrayList;

import static frc.robot.Constants.PathFollowing.kPathLookaheadTime;
import static frc.robot.Constants.SmartDashboardKeys.kTrajectoryThetaPKey;
import static frc.robot.Paths.NewFiveBallRoutine.fiveBall1;
import static frc.robot.RobotContainer.godSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final CumulativeAverage xPositionErrorAverage = new CumulativeAverage();
    private final CumulativeAverage yPositionErrorAverage = new CumulativeAverage();
    private final CumulativeAverage thetaPositionErrorAverage = new CumulativeAverage();
    private HolonomicDriveController holonomicDriveController;

    private boolean modulesReady = false;
    private boolean moduleConfigRequested = false;
    private Rotation2d initialModuleAngle;

    public SwerveTrajectoryCommand(PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
    }

    public void initialize() {
        drivetrain.getConfig().getThetaController().enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        holonomicDriveController = new HolonomicDriveController(
                drivetrain.getConfig().getXController(),
                drivetrain.getConfig().getYController(),
                drivetrain.getConfig().getThetaController());

        holonomicDriveController.setEnabled(true);

        LiveDashboardTable.getInstance().setFollowingPath(true);

        LiveDashboardHelper.putRobotData(godSubsystem.getAllianceSpecificPose());
        LiveDashboardHelper.putTrajectoryData(trajectory.getInitialPose());

        drivetrain.getField().getObject("traj").setTrajectory(trajectory);

        xPositionErrorAverage.clear();
        yPositionErrorAverage.clear();
        thetaPositionErrorAverage.clear();

        modulesReady = false;
        moduleConfigRequested = false;

        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState)
                trajectory.sample(kPathLookaheadTime);

        ChassisSpeeds speeds = holonomicDriveController.calculate(drivetrain.getPoseMeters(),
                state, state.holonomicRotation);

        initialModuleAngle = drivetrain.getSwerveDriveKinematics().toSwerveModuleStates(speeds)[0].angle;
    }

    public void execute() {
        SmartDashboard.putBoolean("Are modules ready", modulesReady);

        if (modulesReady) {
            double thetaP = SmartDashboard.getNumber(kTrajectoryThetaPKey, drivetrain.getConfig().getThetaController().getP());

            drivetrain.getConfig().getThetaController().setP(thetaP);

            double currentTime = timer.get();
            double kXInstantPositionError = drivetrain.getConfig().getXController().getPositionError();
            double kYInstantPositionError = drivetrain.getConfig().getYController().getPositionError();
            double kThetaInstantPositionError = drivetrain.getConfig().getThetaController().getPositionError();

            PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
            ChassisSpeeds speeds = holonomicDriveController.calculate(drivetrain.getPoseMeters(), state, state.holonomicRotation);

//            drivetrain.move(
//                    speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

            drivetrain.faceDirection(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, state.holonomicRotation, false);

            LiveDashboardHelper.putRobotData(godSubsystem.getAllianceSpecificPose());
            LiveDashboardHelper.putTrajectoryData(trajectory.sample(currentTime).poseMeters);

            SmartDashboard.putNumber("kX Position Error", kXInstantPositionError);
            SmartDashboard.putNumber("kY Position Error", kYInstantPositionError);
            SmartDashboard.putNumber("kTheta Position Error", kThetaInstantPositionError);

            xPositionErrorAverage.addData(kXInstantPositionError);
            yPositionErrorAverage.addData(kYInstantPositionError);
            thetaPositionErrorAverage.addData(kThetaInstantPositionError);
        } else if (!moduleConfigRequested) {
            drivetrain.getSwerveModules().forEach(m -> m.setAbsoluteAzimuthRotation2d(initialModuleAngle));

            moduleConfigRequested = true;
        }

        if (moduleAnglesOnTarget() && !modulesReady) {
            drivetrain.getSwerveModules().forEach(WaltSwerveModule::resetLastEncoderReading);
            modulesReady = true;

            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        drivetrain.drive(0.0, 0.0, 0.0);

        SmartDashboard.putNumber("X Error Average", xPositionErrorAverage.getMean());
        SmartDashboard.putNumber("Y Error Average", yPositionErrorAverage.getMean());
        SmartDashboard.putNumber("Theta Error Average", Math.toDegrees(thetaPositionErrorAverage.getMean()));
    }

    private boolean moduleAnglesOnTarget() {
        boolean onTarget = true;

        for(WaltSwerveModule m : drivetrain.getSwerveModules()) {
            double currentAngle = UtilMethods.restrictAngle(m.getAzimuthRotation2d().getDegrees(), -180, 180);
            double targetAngle = UtilMethods.restrictAngle(initialModuleAngle.getDegrees(), -180, 180);

            onTarget &= Math.abs(targetAngle - currentAngle) < 0.25;
        }

        return onTarget;
    }

}
