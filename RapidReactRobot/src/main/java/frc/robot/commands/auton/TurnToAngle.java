package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.UtilMethods;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SmartDashboardKeys.kTurnToAngleErrorDegreesKey;
import static frc.robot.Constants.SmartDashboardKeys.kTurnToAngleOmegaOutputKey;
import static frc.robot.RobotContainer.godSubsystem;

public class TurnToAngle extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final ProfiledPIDController controller = drivetrain.getConfig().getTurnToAngleController();

    private final DoubleSupplier targetHeadingSupplier;
    private double targetHeading;

    public TurnToAngle(DoubleSupplier targetHeadingSupplier) {
        addRequirements(drivetrain);

        this.targetHeadingSupplier = targetHeadingSupplier;
    }

    public TurnToAngle(double targetHeading) {
        this(() -> targetHeading);
    }

    @Override
    public void initialize() {
        targetHeading = UtilMethods.restrictAngle(targetHeadingSupplier.getAsDouble(), -180.0, 180.0);

        controller.reset(new TrapezoidProfile.State(getHeading(),
                drivetrain.getAngularVelocityDegreesPerSec()));

        controller.setGoal(targetHeading);

        System.out.println("Turning to: " + targetHeading);
    }

    @Override
    public void execute() {
        double turnRate = controller.calculate(getHeading(), targetHeading);

        turnRate += Math.signum(turnRate) * drivetrain.getConfig().getMinTurnOmega();

        SmartDashboard.putNumber(kTurnToAngleErrorDegreesKey, controller.getPositionError());
        SmartDashboard.putNumber(kTurnToAngleOmegaOutputKey, turnRate);

        drivetrain.move(0, 0, turnRate, false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.move(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return UtilMethods.isWithinTolerance(getHeading(), targetHeading, 0.5);
    }

    private double getHeading() {
        return UtilMethods.restrictAngle(drivetrain.getHeading().getDegrees(), -180, 180);
    }

}

