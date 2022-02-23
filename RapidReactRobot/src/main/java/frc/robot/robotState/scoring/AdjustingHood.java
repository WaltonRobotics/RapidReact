package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.FieldConstants.kHoodCloseUpDistanceFeet;
import static frc.robot.OI.barfButton;
import static frc.robot.RobotContainer.godSubsystem;

public class AdjustingHood implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        if (LimelightHelper.getDistanceToTargetFeet() <= kHoodCloseUpDistanceFeet || barfButton.get()) {
            shooter.setHoodPosition(Shooter.HoodPosition.SIXTY_DEGREES);
        } else {
            shooter.setHoodPosition(Shooter.HoodPosition.SEVENTY_DEGREES);
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get() && !barfButton.get()) {
            return new ScoringMode();
        }

        godSubsystem.handleTransportConveyorManualOverride();
        godSubsystem.handleFeedConveyorManualOverride();

        if (barfButton.get()) {
            return new PreparingToShoot();
        }

        return new AligningAndSpinningUp();
    }

    @Override
    public void finish() {

    }

}
