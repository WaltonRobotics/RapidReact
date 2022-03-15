package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.DriverPreferences.kMaxTranslationalAccelerationMsecSquared;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final SlewRateLimiter vxRateLimiter;
    private final SlewRateLimiter vyRateLimiter;

    private static boolean enabled = true;
    private boolean isFieldRelative = true;
    private boolean isPositionalRotation = false;

    public DriveCommand() {
        addRequirements(drivetrain);

        SmartDashboard.putNumber("Minimum omega command", 0.1);

        vxRateLimiter = new SlewRateLimiter(kMaxTranslationalAccelerationMsecSquared);
        vyRateLimiter = new SlewRateLimiter(kMaxTranslationalAccelerationMsecSquared);
    }

    public static void setIsEnabled(boolean isEnabled) {
        enabled = isEnabled;
    }

    @Override
    public void execute() {
        if (enabled) {
            if (toggleFieldRelativeMode.isRisingEdge()) {
                isFieldRelative = !isFieldRelative;
            }

            if (toggleRotationMode.isRisingEdge()) {
                isPositionalRotation = !isPositionalRotation;
            }

            double forward = OI.forwardScale.apply(getForward());
            double strafe = OI.strafeScale.apply(getStrafe());

            double vx = vxRateLimiter.calculate(forward * drivetrain.getConfig().getMaxSpeedMetersPerSecond());
            double vy = vyRateLimiter.calculate(strafe * drivetrain.getConfig().getMaxSpeedMetersPerSecond());

            // Limit movement when climbing
            if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.CLIMBING_MODE && faceClimb.get()) {
                drivetrain.faceDirection(vx, vy, Rotation2d.fromDegrees(180.0), isFieldRelative);
            } else if (faceClosest.get()) {
                drivetrain.faceClosest(vx, vy, isFieldRelative);
            } else if (isPositionalRotation && godSubsystem.getCurrentMode() == Superstructure.CurrentMode.SCORING_MODE) {
                double rotateX = getRotateX() * 10;
                double rotateY = getRotateY() * 10;

                if (Math.abs(rotateX) > 1 || Math.abs(rotateY) > 1) {
                    drivetrain.faceDirection(vx, vy,
                            new Rotation2d(Math.atan2(rotateY, rotateX)).minus(Rotation2d.fromDegrees(90.0)),
                            isFieldRelative);
                } else {
                    drivetrain.move(vx, vy, 0, isFieldRelative);
                }
            } else {
                double yaw = OI.yawScale.apply(getRotateX());
                double omega = 0;

                // Ensure at least the minimum turn omega is supplied to the drivetrain to prevent stalling
                if (Math.abs(getRotateX()) > yawScale.getDeadband()) {
                    omega = Math.signum(yaw) * Math.max(Math.abs(yaw * drivetrain.getConfig().getMaxOmega()),
                            drivetrain.getConfig().getMinTurnOmega());
                }

                // Limit movement when climbing
                if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.CLIMBING_MODE) {
                    vx = UtilMethods.limitMagnitude(vx, drivetrain.getConfig().getClimbingMaxMetersPerSecond());
                    vy = UtilMethods.limitMagnitude(vy, drivetrain.getConfig().getClimbingMaxMetersPerSecond());
                    omega = UtilMethods.limitMagnitude(omega, drivetrain.getConfig().getClimbingMaxOmega());
                }

                drivetrain.move(vx, vy, omega, isFieldRelative);
//            drivetrain.move(0, 0, SmartDashboard.getNumber("Minimum omega command", 0.1), true);
            }
        }
    }

    public double getForward() {
        return -driveGamepad.getLeftY();
    }

    public double getStrafe() {
        return -driveGamepad.getLeftX();
    }

    public double getRotateX() {
        return -driveGamepad.getRightX();
    }

    public double getRotateY() {
        return -driveGamepad.getRightY();
    }

}
