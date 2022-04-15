package frc.robot.robotState.climbing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.kDefaultExtensionAcceleration;
import static frc.robot.Constants.Climber.kDefaultExtensionCruiseVelocity;
import static frc.robot.OI.overrideNextClimbStateButton;
import static frc.robot.OI.stopClimbButton;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Climber.ClimberExtensionPosition.FINALIZE_HIGH_BAR_CLIMB_LENGTH;
import static frc.robot.subsystems.Climber.ClimberPivotPosition.STOWED_ANGLE;

public class PullOntoHighBar implements IState {

    private final Target pullUpLength = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.FINALIZE_HIGH_BAR_CLIMB_LENGTH);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setPivotPositionDemand(STOWED_ANGLE);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(FINALIZE_HIGH_BAR_CLIMB_LENGTH);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);

        godSubsystem.getClimber().configExtensionSmartMotion(kDefaultExtensionCruiseVelocity,
                kDefaultExtensionAcceleration);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if ((pullUpLength.isWithinTolerance(extensionHeight))
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        godSubsystem.handleExtensionManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
