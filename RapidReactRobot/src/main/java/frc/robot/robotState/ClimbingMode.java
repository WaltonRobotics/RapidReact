package frc.robot.robotState;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.OI;
import frc.robot.config.Target;
import frc.robot.robotState.climbing.HighBarClimbPullUpToMidBar;
import frc.robot.robotState.climbing.MidBarClimbPullUp;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Superstructure;

import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingMode implements IState {

    private Target stowedAngle = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.LINING_UP_FOR_MID_BAR);
    private Target hookingLength = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.MID_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH);

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.CLIMBING_MODE);

        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.LINING_UP_FOR_MID_BAR);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.MID_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH);

        godSubsystem.getClimber().enableExtensionLowerLimit();
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (OI.toggleBetweenScoringAndClimbingModeButton.isRisingEdge()) {
            return new ScoringModeTransition();
        }

        if (selectMidRungButton.isRisingEdge()) {
            godSubsystem.setSelectedRung(Superstructure.ClimbingTargetRung.MID_RUNG);
            stowedAngle = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.LINING_UP_FOR_MID_BAR);
            godSubsystem.getClimber().setPivotPositionDemand(stowedAngle);

            hookingLength = currentRobot.getExtensionTarget(
                    Climber.ClimberExtensionPosition.MID_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH);
            godSubsystem.getClimber().setExtensionPositionDemand(hookingLength);
        } else if (selectHighRungButton.isRisingEdge()) {
            godSubsystem.setSelectedRung(Superstructure.ClimbingTargetRung.HIGH_RUNG);
            stowedAngle = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.ANGLE_HOOK_THETA_FOR_MID_BAR);
            godSubsystem.getClimber().setPivotPositionDemand(stowedAngle);

            hookingLength = currentRobot.getExtensionTarget(
                    Climber.ClimberExtensionPosition.HIGH_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH);
            godSubsystem.getClimber().setExtensionPositionDemand(hookingLength);
        }

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if ((stowedAngle.isWithinTolerance(pivotAngle) && hookingLength.isWithinTolerance(extensionHeight)
                && advanceClimbingProcessButton.get()) || overrideNextClimbStateButton.isRisingEdge()) {
            if (godSubsystem.getSelectedRung() == Superstructure.ClimbingTargetRung.MID_RUNG) {
                return new MidBarClimbPullUp();
            } else {
                return new HighBarClimbPullUpToMidBar();
            }
        }

        godSubsystem.handleExtensionManualOverride();
        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {
//        godSubsystem.getClimber().configExtensionSmartMotion(kDefaultExtensionCruiseVelocity,
//                kDefaultExtensionAcceleration);
    }

}
