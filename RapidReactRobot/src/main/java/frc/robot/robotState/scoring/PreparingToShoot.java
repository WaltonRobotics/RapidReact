package frc.robot.robotState.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.ContextFlags.kIsInShooterTuningMode;
import static frc.robot.Constants.FieldConstants.kHoodCloseUpDistanceFeet;
import static frc.robot.Constants.Shooter.kBarfVelocityRawUnits;
import static frc.robot.Constants.Shooter.kDefaultVelocityRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.kShooterTuningSetpointVelocityNUKey;
import static frc.robot.OI.barfButton;
import static frc.robot.RobotContainer.godSubsystem;

public class PreparingToShoot implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        if (barfButton.get()) {
            godSubsystem.setCurrentTargetFlywheelVelocity(kBarfVelocityRawUnits);
        } else if (kIsInShooterTuningMode) {
            shooter.setAdjustableHoodDutyCycleDemand(SmartDashboard.getNumber("Hood angle setpoint", 0.0));

            godSubsystem.setCurrentTargetFlywheelVelocity(
                    SmartDashboard.getNumber(kShooterTuningSetpointVelocityNUKey, kDefaultVelocityRawUnits));
        } else {
            // Re-adjust hood
            if (LimelightHelper.getDistanceToTargetFeet() <= kHoodCloseUpDistanceFeet) {
                shooter.setHoodPosition(Shooter.HoodPosition.SIXTY_DEGREES);
            } else {
                shooter.setHoodPosition(Shooter.HoodPosition.SEVENTY_DEGREES);
            }

            // Recalculate target velocity
            godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        godSubsystem.handleIntakingAndOuttaking();

        return new SpinningUp();
    }

    @Override
    public void finish() {

    }

}
