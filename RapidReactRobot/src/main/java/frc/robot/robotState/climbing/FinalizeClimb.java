package frc.robot.robotState.climbing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.config.LimitPair;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.godSubsystem;

public class FinalizeClimb implements IState {

    @Override
    public void initialize() {
        double currentPivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();
        double currentExtensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        LimitPair pivotLimits = new LimitPair(currentPivotAngle + 50, currentPivotAngle - 50);
        LimitPair extensionLimits = new LimitPair(currentExtensionHeight + 50, currentExtensionHeight - 50);

        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemandNU(currentPivotAngle);
        godSubsystem.getClimber().setPivotLimits(pivotLimits);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemandNU(currentExtensionHeight);
        godSubsystem.getClimber().setExtensionLimits(extensionLimits);

        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Brake);

        // Energize disc brake
        godSubsystem.getClimber().setClimberDiscBrakeStateDemand(true);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
