package frc.robot.robotState.climbing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.Climber.kDeployHighBarArmsAngleDegrees;
import static frc.robot.Constants.Climber.kTransferPercentOutput;
import static frc.robot.Constants.SmartDashboardKeys.kClimberDeployHighBarArmsAngleKey;
import static frc.robot.OI.overrideNextClimbStateButton;
import static frc.robot.OI.stopClimbButton;
import static frc.robot.RobotContainer.godSubsystem;

public class PullUpOntoMidBar implements IState {

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);
        godSubsystem.getClimber().releaseExtensionLowerLimit();
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        double targetPitch = SmartDashboard.getNumber(kClimberDeployHighBarArmsAngleKey, kDeployHighBarArmsAngleDegrees);

        if (((godSubsystem.getClimber().isLeftExtensionLowerLimitClosed()
                && godSubsystem.getClimber().isRightExtensionLowerLimitClosed())
//                && UtilMethods.isWithinTolerance(godSubsystem.getDrivetrain().getPitch().getDegrees(), targetPitch, 1)
                // godSubsystem.getDrivetrain().isOnFrontSwing())
                || overrideNextClimbStateButton.isRisingEdge())) {
            return new DeployHighBarArms();
        }

        if (!godSubsystem.getClimber().isLeftExtensionLowerLimitClosed()
                && !godSubsystem.getClimber().isRightExtensionLowerLimitClosed()) {
            godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);
            godSubsystem.getClimber().setExtensionPercentOutputDemand(kTransferPercentOutput);
        } else {
            godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
            godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
        }

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().enableExtensionLowerLimit();
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
    }

}
