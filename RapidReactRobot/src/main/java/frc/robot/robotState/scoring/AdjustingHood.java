package frc.robot.robotState.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.robotState.ScoringModeTransition;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ContextFlags.kIsInShooterTuningMode;
import static frc.robot.Constants.Shooter.kBarfHoodAngle;
import static frc.robot.Constants.SmartDashboardKeys.kShooterHoodPositionSetpointKey;
import static frc.robot.OI.barfButton;
import static frc.robot.OI.overrideAutoAimAndShootButton;
import static frc.robot.RobotContainer.godSubsystem;

public class AdjustingHood implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setTransportDemand(0.0);
        godSubsystem.getConveyor().setFeedDemand(0.0);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        if (!kIsInShooterTuningMode) {
//            if (LimelightHelper.getDistanceToTargetFeet() <= kHoodCloseUpDistanceFeet || barfButton.get()) {
//                shooter.setHoodPosition(Shooter.HoodPosition.SEVENTY_DEGREES);
//            } else {
//                shooter.setHoodPosition(Shooter.HoodPosition.SIXTY_DEGREES);
//            }

            if (barfButton.get()) {
                shooter.setAdjustableHoodDutyCycleDemand(kBarfHoodAngle);
            } else {
                shooter.setAdjustableHoodDutyCycleDemand(shooter.getEstimatedHoodAngleFromTarget());
            }
        } else {
            shooter.setAdjustableHoodDutyCycleDemand(
                    SmartDashboard.getNumber(kShooterHoodPositionSetpointKey, 0.0));
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get() && !barfButton.get() && !overrideAutoAimAndShootButton.get()
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToShoot()))
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToAlignAndShoot()))) {
            return new ScoringModeTransition();
        }

        godSubsystem.handleIntakingAndOuttaking();

        if (barfButton.get()) {
            return new PreparingToShoot();
        }

        if ((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToShoot())
                || kIsInShooterTuningMode
                || overrideAutoAimAndShootButton.get()) {
            return new NudgingDown();
        }

        godSubsystem.handleIdleSpinUp();

        return new AligningAndSpinningUp();
    }

    @Override
    public void finish() {

    }

}
