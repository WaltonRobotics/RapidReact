package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.RobotContainer.*;

public class AimCommandLime extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final double minOmega = 0.6;

    private final PIDController controller = new PIDController(0.05, 0.015, 0.000);

    public AimCommandLime() {
        addRequirements(drivetrain);

        SmartDashboard.putData(kLimelightAlignControllerKey, controller);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        double headingError = -LimelightHelper.getTX();
        double turnRate = controller.calculate(headingError, 0.0);

        turnRate += Math.signum(turnRate) * minOmega;

        SmartDashboard.putNumber(kLimelightAlignErrorDegrees, controller.getPositionError());
        SmartDashboard.putNumber(kLimelightAlignOmegaOutputKey, turnRate);

        drivetrain.move(0,0, turnRate,false);
    }

}
