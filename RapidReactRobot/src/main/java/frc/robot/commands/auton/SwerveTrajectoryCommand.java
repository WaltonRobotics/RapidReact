package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
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

import static frc.robot.Constants.SmartDashboardKeys.kTrajectoryThetaPKey;
import static frc.robot.RobotContainer.godSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final CumulativeAverage xPositionErrorAverage = new CumulativeAverage();
    private final CumulativeAverage yPositionErrorAverage = new CumulativeAverage();
    private final CumulativeAverage thetaPositionErrorAverage = new CumulativeAverage();
    private HolonomicDriveController holonomicDriveController;
    private final Rotation2d initialModuleAngle;
    private boolean waitUntilModulesReady;
    private boolean areModulesReady = false;

    public SwerveTrajectoryCommand(PathPlannerTrajectory trajectory) {
        this(trajectory, Rotation2d.fromDegrees(0), false);
    }

    public SwerveTrajectoryCommand(PathPlannerTrajectory trajectory, Rotation2d initialModuleAngle, boolean waitUntilModulesReady) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
        this.initialModuleAngle = initialModuleAngle;
        this.waitUntilModulesReady = waitUntilModulesReady;
    }

    public void initialize() {
        drivetrain.getConfig().getThetaController().enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        holonomicDriveController = new HolonomicDriveController(
                drivetrain.getConfig().getXController(),
                drivetrain.getConfig().getYController(),
                drivetrain.getConfig().getThetaController());

        holonomicDriveController.setEnabled(true);

        LiveDashboardTable.getInstance().setFollowingPath(true);

        LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
        LiveDashboardHelper.putTrajectoryData(trajectory.getInitialPose());

        drivetrain.getField().getObject("traj").setTrajectory(trajectory);

        timer.reset();
        timer.start();

        xPositionErrorAverage.clear();
        yPositionErrorAverage.clear();
        thetaPositionErrorAverage.clear();

        areModulesReady = !waitUntilModulesReady;
    }

    public void execute() {
        SmartDashboard.putBoolean("Are modules ready", areModulesReady);

        if (!areModulesReady) {
            boolean areAllModulesAligned = true;

            ArrayList<WaltSwerveModule> modules = drivetrain.getSwerveModules();

            for (WaltSwerveModule module : modules) {
                module.setAbsoluteAzimuthRotation2d(initialModuleAngle);

                double moduleAngle = UtilMethods.restrictAngle(module.getAzimuthRotation2d().getDegrees(),
                        -180.0, 180.0);

                double desiredAngle = UtilMethods.restrictAngle(initialModuleAngle.getDegrees(),
                        -180.0, 180.0);

                if (!UtilMethods.isWithinTolerance(moduleAngle, desiredAngle, 5)) {
                    areAllModulesAligned = false;
                }
            }

            areModulesReady = areAllModulesAligned;

            if (areModulesReady) {
                timer.reset();
                timer.start();

                modules.forEach(WaltSwerveModule::resetLastEncoderReading);
            }
        } else {
            double thetaP = SmartDashboard.getNumber(kTrajectoryThetaPKey, drivetrain.getConfig().getThetaController().getP());

            drivetrain.getConfig().getThetaController().setP(thetaP);

            double currentTime = timer.get();
            double kXInstantPositionError = drivetrain.getConfig().getXController().getPositionError();
            double kYInstantPositionError = drivetrain.getConfig().getYController().getPositionError();
            double kThetaInstantPositionError = drivetrain.getConfig().getThetaController().getPositionError();

            PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
            ChassisSpeeds speeds = holonomicDriveController.calculate(drivetrain.getPoseMeters(), state, state.holonomicRotation);

            drivetrain.move(
                    speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

//            drivetrain.faceDirection(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, state.holonomicRotation, false);

            LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
            LiveDashboardHelper.putTrajectoryData(trajectory.sample(currentTime).poseMeters);

            SmartDashboard.putNumber("kX Position Error", kXInstantPositionError);
            SmartDashboard.putNumber("kY Position Error", kYInstantPositionError);
            SmartDashboard.putNumber("kTheta Position Error", kThetaInstantPositionError);

            xPositionErrorAverage.addData(kXInstantPositionError);
            yPositionErrorAverage.addData(kYInstantPositionError);
            thetaPositionErrorAverage.addData(kThetaInstantPositionError);
        }
    }

    @Override
    public boolean isFinished() {
        return areModulesReady && timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        drivetrain.drive(0.0, 0.0, 0.0);

        SmartDashboard.putNumber("X Error Average", xPositionErrorAverage.getMean());
        SmartDashboard.putNumber("Y Error Average", yPositionErrorAverage.getMean());
        SmartDashboard.putNumber("Theta Error Average", Math.toDegrees(thetaPositionErrorAverage.getMean()));
    }

}
