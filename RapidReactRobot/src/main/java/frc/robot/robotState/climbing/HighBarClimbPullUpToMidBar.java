package frc.robot.robotState.climbing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.Climber.kDeployHighBarArmsAngleDegrees;
import static frc.robot.Constants.SmartDashboardKeys.kClimberDeployHighBarArmsAngleKey;
import static frc.robot.OI.overrideNextClimbStateButton;
import static frc.robot.OI.stopClimbButton;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Climber.ClimberPivotPosition.STOWED_ANGLE;

public class HighBarClimbPullUpToMidBar implements IState {

    private final Target pullUpLength = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.CLOSE_IN_TO_ZERO_LENGTH);

    private final Timer timer = new Timer();

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setPivotPositionDemand(STOWED_ANGLE);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.CLOSE_IN_TO_ZERO_LENGTH);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);

//        godSubsystem.getClimber().setHighBarArmsDeployed(true);

        godSubsystem.getClimber().configExtensionSmartMotion(20000, 40000);

        timer.reset();
        timer.start();
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            System.out.println("Climb Finalized");
            return new FinalizeClimb();
        }

        if (timer.hasElapsed(0.25)) {
            godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Coast);
        }

        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        double targetPitch = SmartDashboard.getNumber(kClimberDeployHighBarArmsAngleKey, kDeployHighBarArmsAngleDegrees);

//        if ((UtilMethods.isWithinTolerance(godSubsystem.getDrivetrain().getPitch().getDegrees(), targetPitch, 1)
//                && godSubsystem.getDrivetrain().isOnFrontSwing())) {
//            return new DeployHighBarArms();
//        }

        if ((pullUpLength.isWithinTolerance(extensionHeight))
                || overrideNextClimbStateButton.isRisingEdge()) {
            System.out.println("Entering Pull Up Onto Mid Bar state");
            var rungState = godSubsystem.getSelectedRung();
            return new PullUpOntoMidBar();
        }

        godSubsystem.handleExtensionManualOverride();

        return this;
    }

    @Override
    public void finish() {
        DriveCommand.setIsEnabled(false);
    }

}
