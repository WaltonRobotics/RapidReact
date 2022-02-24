package frc.robot.robotState.climbing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.config.LimitPair;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;
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

        driveGamepad.setRumble(kLeftRumble, 1); //left and right rumble types, 0 to 1 value
        driveGamepad.setRumble(kRightRumble, 1);
        manipulationGamepad.setRumble(kLeftRumble,1);
        manipulationGamepad.setRumble(kRightRumble, 1);

    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        driveGamepad.setRumble(kLeftRumble, 0.5); //left and right rumble types, 0 to 1 value
        driveGamepad.setRumble(kRightRumble, 0.5 );
        manipulationGamepad.setRumble(kLeftRumble,0.5);
        manipulationGamepad.setRumble(kRightRumble, 0.5);

        return this;
    }

    @Override
    public void finish() {
        driveGamepad.setRumble(kLeftRumble, 0); //left and right rumble types, 0 to 1 value
        driveGamepad.setRumble(kRightRumble, 0 );
        manipulationGamepad.setRumble(kLeftRumble,0);
        manipulationGamepad.setRumble(kRightRumble, 0);

    }

}
