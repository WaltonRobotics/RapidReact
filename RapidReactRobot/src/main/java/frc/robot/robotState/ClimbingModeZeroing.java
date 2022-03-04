package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingModeZeroing implements IState {

    private double timeout;

    @Override
    public void initialize() {
        // Set pivot to hold current position
        // Limit pivot ROM
        // Zero extension
        // Limit extension ROM

        double currentPivotAngle = godSubsystem.getClimber().getPivotAbsoluteEncoderPositionNU();

        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemandNU(currentPivotAngle);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.ZEROING);
        godSubsystem.getClimber().setZeroed(false);
        godSubsystem.getClimber().releaseExtensionLowerLimit();
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        
        if (godSubsystem.getClimber().isLeftExtensionLowerLimitClosed()
                || godSubsystem.getClimber().isRightExtensionLowerLimitClosed()) {
            godSubsystem.getClimber().setZeroed(true);
        } else {
            timeout = godSubsystem.getCurrentTime() + 0.02;
        }

        if (godSubsystem.getClimber().isZeroed()) {
            if (godSubsystem.getCurrentTime() >= timeout) {
                return new ClimbingMode();
            }
        }

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().enableExtensionLowerLimit();
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

}
