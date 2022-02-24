package frc.robot.robotState.climbing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.config.LimitPair;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;
import static frc.robot.Constants.DriverPreferences.kFinishedClimbingRumbleValue;
import static frc.robot.OI.driveGamepad;
import static frc.robot.OI.manipulationGamepad;
import static frc.robot.RobotContainer.godSubsystem;

public class FinalizeClimb implements IState {

    @Override
    public void initialize() {
        double currentPivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();
        double currentExtensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        LimitPair pivotLimits = new LimitPair(currentPivotAngle - 50, currentPivotAngle + 50);
        LimitPair extensionLimits = new LimitPair(currentExtensionHeight - 50, currentExtensionHeight + 50);

        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemandNU(currentPivotAngle);
        godSubsystem.getClimber().setPivotLimits(pivotLimits);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemandNU(currentExtensionHeight);
        godSubsystem.getClimber().setExtensionLimits(extensionLimits);

        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Brake);

        // Energize disc brake and climber locks
        godSubsystem.getClimber().setLeftClimberLockStateDemand(false);
        godSubsystem.getClimber().setRightClimberLockStateDemand(false);
        godSubsystem.getClimber().setClimberDiscBrakeStateDemand(false);

        driveGamepad.setRumble(kLeftRumble, kFinishedClimbingRumbleValue);
        driveGamepad.setRumble(kRightRumble, kFinishedClimbingRumbleValue);

        manipulationGamepad.setRumble(kLeftRumble, kFinishedClimbingRumbleValue);
        manipulationGamepad.setRumble(kRightRumble, kFinishedClimbingRumbleValue);
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
        driveGamepad.setRumble(kLeftRumble, 0);
        driveGamepad.setRumble(kRightRumble, 0);

        manipulationGamepad.setRumble(kLeftRumble, 0);
        manipulationGamepad.setRumble(kRightRumble, 0);
    }

}
