package frc.robot.robotState.climbing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.kTransferPercentOutput;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Climber.ClimberPivotPosition.STOWED_ANGLE;

public class DeployHighBarArms implements IState {

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(new Target(godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU()));
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);
        godSubsystem.getClimber().releaseExtensionLowerLimit();

        godSubsystem.getClimber().setHighBarArmsDeployed(true);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        if (advanceClimbingProcessButton.isRisingEdge()) {
            return new PullOntoHighBar();
        }

//        godSubsystem.handleExtensionManualOverride();

        if (!godSubsystem.getClimber().isLeftExtensionLowerLimitClosed()
                && !godSubsystem.getClimber().isRightExtensionLowerLimitClosed()) {
            godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);
            godSubsystem.getClimber().setExtensionPercentOutputDemand(kTransferPercentOutput);
        } else {
            godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
            godSubsystem.getClimber().enableExtensionLowerLimit();
            godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
        }

        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().enableExtensionLowerLimit();
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.STOWED);
    }

}
