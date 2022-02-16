package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.CumulativeAverage;

import static frc.robot.Constants.SwerveDriveConfig.*;
import static frc.robot.RobotContainer.godSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private HolonomicDriveController holonomicDriveController;

    private final CumulativeAverage xPositionErrorAverage = new CumulativeAverage();
    private final CumulativeAverage yPositionErrorAverage = new CumulativeAverage();
    private final CumulativeAverage thetaPositionErrorAverage = new CumulativeAverage();

    public SwerveTrajectoryCommand(PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
        SmartDashboard.putNumber("thetaP", 3);
    }

    public void initialize() {
        kThetaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        holonomicDriveController = new HolonomicDriveController(kXController, kYController, kThetaController);

        holonomicDriveController.setEnabled(true);

        timer.reset();
        timer.start();

        LiveDashboardTable.getInstance().setFollowingPath(true);

        LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
        LiveDashboardHelper.putTrajectoryData(trajectory.getInitialPose());

        drivetrain.getField().getObject("traj").setTrajectory(trajectory);
    }

    public void execute() {
        double thetaP = SmartDashboard.getNumber("thetaP", 3);
        kThetaController.setP(thetaP);
        double currentTime = timer.get();
        double kXInstantPositionError = kXController.getPositionError();
        double kYInstantPositionError = kYController.getPositionError();
        double kThetaInstantPositionError = kThetaController.getPositionError();

        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
        ChassisSpeeds speeds = holonomicDriveController.calculate(drivetrain.getPoseMeters(), state, state.holonomicRotation);

        drivetrain.move(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

        LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
        LiveDashboardHelper.putTrajectoryData(trajectory.sample(currentTime).poseMeters);
        
        SmartDashboard.putNumber("kX Position Error", kXInstantPositionError);
        SmartDashboard.putNumber("kY Position Error", kYInstantPositionError);
        SmartDashboard.putNumber("kTheta Position Error", kThetaInstantPositionError);


        xPositionErrorAverage.addData(kXInstantPositionError);
        yPositionErrorAverage.addData(kYInstantPositionError);
        thetaPositionErrorAverage.addData(kThetaInstantPositionError);
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
        SmartDashboard.putNumber("Theta Error Average", thetaPositionErrorAverage.getMean());
    }

}
