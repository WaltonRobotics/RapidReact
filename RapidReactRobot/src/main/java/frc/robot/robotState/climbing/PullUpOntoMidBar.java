package frc.robot.robotState.climbing;

import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.Climber.kTransferPercentOutput;
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

        if ((UtilMethods.isWithinTolerance(godSubsystem.getDrivetrain().getPitch().getDegrees(), 44, 1)
                && godSubsystem.getDrivetrain().isOnFrontSwing())
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new DeployHighBarArms();
        }

        if (!godSubsystem.getClimber().isLeftExtensionLowerLimitClosed()
                && !godSubsystem.getClimber().isRightExtensionLowerLimitClosed()) {
            godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);
            godSubsystem.getClimber().setExtensionPercentOutputDemand(kTransferPercentOutput);
        } else {
            godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
            godSubsystem.getClimber().enableExtensionLowerLimit();
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
