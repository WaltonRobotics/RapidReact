package frc.robot.robotState.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;
import frc.robot.vision.ColorSensor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.ContextFlags.kIsInShooterTuningMode;
import static frc.robot.Constants.DriverPreferences.kUseAutoReject;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SmartDashboardKeys.kShooterTuningSetpointVelocityNUKey;
import static frc.robot.OI.barfButton;
import static frc.robot.RobotContainer.godSubsystem;

public class PreparingToShoot implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        // Disable driver control to lock drivetrain in place
        DriveCommand.setIsEnabled(false);

        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        LimelightHelper.takeSnapshot();

        if (barfButton.get() || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToBarf())) {
            if (godSubsystem.isInAuton()) {
                godSubsystem.setCurrentTargetFlywheelVelocity(kAutonBarfVelocityRawUnits);
            } else {
                godSubsystem.setCurrentTargetFlywheelVelocity(kBarfVelocityRawUnits);
            }
        } else if (kIsInShooterTuningMode) {
            godSubsystem.setCurrentTargetFlywheelVelocity(
                    SmartDashboard.getNumber(kShooterTuningSetpointVelocityNUKey, kDefaultVelocityRawUnits));
        }
        else {
            // Re-adjust hood
//            if (LimelightHelper.getDistanceToTargetFeet() <= kHoodCloseUpDistanceFeet) {
//                shooter.setHoodPosition(Shooter.HoodPosition.SEVENTY_DEGREES);
//            } else {
//                shooter.setHoodPosition(Shooter.HoodPosition.SIXTY_DEGREES);
//            }

            // Recalculate target velocity
            LimelightHelper.takeSnapshot();

            godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.needsToAutoReject()) {
            shooter.setFlywheelDemand(kBarfVelocityRawUnits);
        } else {
            shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());
        }

        godSubsystem.handleIntakingAndOuttaking();

        godSubsystem.getDrivetrain().xLockSwerveDrive();

        return new SpinningUp();
    }

    @Override
    public void finish() {

    }

}
