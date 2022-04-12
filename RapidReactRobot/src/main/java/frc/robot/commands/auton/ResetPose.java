package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.RobotContainer.godSubsystem;

public class ResetPose extends InstantCommand {

    public ResetPose(PathPlannerTrajectory trajectory) {
        super(() ->
                godSubsystem.getDrivetrain().resetPose(trajectory.getInitialPose(), trajectory.getInitialState()));
    }

}
