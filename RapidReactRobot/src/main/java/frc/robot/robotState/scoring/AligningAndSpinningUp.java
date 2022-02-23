package frc.robot.robotState.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.SmartDashboardKeys.kLimelightAlignErrorDegrees;
import static frc.robot.Constants.SmartDashboardKeys.kLimelightAlignOmegaOutputKey;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPINNING_UP_SLOT;

public class AligningAndSpinningUp implements IState {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();
    private final PIDController controller = drivetrain.getConfig().getAutoAlignController();
    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        shooter.setSelectedProfileSlot(SPINNING_UP_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());

        // Disable manual drive control
        DriveCommand.setIsEnabled(false);

        controller.reset();
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get()) {
            return new ScoringMode();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        double headingError = LimelightHelper.getTX();
        double turnRate = controller.calculate(headingError, 0.0);

        turnRate += Math.signum(turnRate) * drivetrain.getConfig().getMinAutoAlignOmega();

        SmartDashboard.putNumber(kLimelightAlignErrorDegrees, controller.getPositionError());
        SmartDashboard.putNumber(kLimelightAlignOmegaOutputKey, turnRate);

        drivetrain.move(0, 0, turnRate, false);

        return new PreparingToShoot();
    }

    @Override
    public void finish() {
        DriveCommand.setIsEnabled(true);

        drivetrain.move(0, 0, 0, false);
    }

}
