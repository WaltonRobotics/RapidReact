package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SwerveDriveConfig.*;
import static frc.robot.RobotContainer.godSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private HolonomicDriveController holonomicDriveController;

    public SwerveTrajectoryCommand(PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
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
        double currentTime = timer.get();

        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
        ChassisSpeeds speeds = holonomicDriveController.calculate(drivetrain.getPoseMeters(), state, state.holonomicRotation);

        drivetrain.move(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

        LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
        LiveDashboardHelper.putTrajectoryData(trajectory.sample(currentTime).poseMeters);
        
        SmartDashboard.putNumber("kX Position Error", kXController.getPositionError());
        SmartDashboard.putNumber("kY Position Error", kYController.getPositionError());
        SmartDashboard.putNumber("kTheta Position Error", kThetaController.getPositionError());
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        drivetrain.drive(0.0, 0.0, 0.0);
    }

}
