package frc.robot.robotState.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;
import frc.robot.util.UtilMethods;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.SmartDashboardKeys.kLimelightAlignErrorDegrees;
import static frc.robot.Constants.SmartDashboardKeys.kLimelightAlignOmegaOutputKey;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPINNING_UP_SLOT;

public class AligningAndSpinningUp implements IState {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();
    private final PIDController controller = drivetrain.getConfig().getAutoAlignController();
    private final Shooter shooter = godSubsystem.getShooter();

    private double timeout;

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        LimelightHelper.setLEDMode(true);
        LimelightHelper.setPipeline(kAlignmentPipeline);

        shooter.setSelectedProfileSlot(SPINNING_UP_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());

        // Disable manual drive control
        DriveCommand.setIsEnabled(false);

        controller.reset();

        timeout = godSubsystem.getCurrentTime() + kAlignmentTimeoutSeconds;
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get()&& !OI.barfButtonButton.get()) {
            return new ScoringMode();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        double headingError = LimelightHelper.getTX();
        double turnRate = controller.calculate(headingError, 0.0);

        turnRate += Math.signum(turnRate) * drivetrain.getConfig().getMinTurnOmega();

        SmartDashboard.putNumber(kLimelightAlignErrorDegrees, controller.getPositionError());
        SmartDashboard.putNumber(kLimelightAlignOmegaOutputKey, turnRate);

        drivetrain.move(0, 0, turnRate, false);

        godSubsystem.handleTransportConveyorManualOverride();
        godSubsystem.handleFeedConveyorManualOverride();

        if (UtilMethods.isWithinTolerance(headingError, 0, kAlignmentToleranceDegrees)
                || godSubsystem.getCurrentTime() >= timeout) {
            return new PreparingToShoot();
        }

        return this;
    }

    @Override
    public void finish() {
        DriveCommand.setIsEnabled(true);

        drivetrain.move(0, 0, 0, false);
    }

}
