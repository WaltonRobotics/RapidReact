package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.SmartDashboardKeys.kDrivetrainIsFieldRelativeKey;
import static frc.robot.Constants.SmartDashboardKeys.kDrivetrainIsPositionalRotationKey;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private static boolean enabled = true;
    private static boolean isFieldRelative = true;
    private boolean isPositionalRotation = false;

    public DriveCommand() {
        addRequirements(drivetrain);

        SmartDashboard.putNumber("Minimum omega command", 0.1);
    }

    public static void setIsEnabled(boolean isEnabled) {
        enabled = isEnabled;
    }

    public static boolean isFieldRelative() {
        return isFieldRelative;
    }

    @Override
    public void execute() {
        if (enabled && !godSubsystem.isInAuton() && !godSubsystem.isInPitCheckMode()) {
            if (toggleFieldRelativeModeButton.isRisingEdge()) {
                isFieldRelative = !isFieldRelative;
            }

            if (toggleRotationModeButton.isRisingEdge()) {
                isPositionalRotation = !isPositionalRotation;
            }

            SmartDashboard.putBoolean(kDrivetrainIsFieldRelativeKey, isFieldRelative);
            SmartDashboard.putBoolean(kDrivetrainIsPositionalRotationKey, isPositionalRotation);

            double vx = godSubsystem.getVx();
            double vy = godSubsystem.getVy();
            double omega = godSubsystem.getOmega();

            // Limit movement when climbing
            if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.CLIMBING_MODE) {
                vx = UtilMethods.limitMagnitude(vx, drivetrain.getConfig().getClimbingMaxMetersPerSecond());
                vy = UtilMethods.limitMagnitude(vy, drivetrain.getConfig().getClimbingMaxMetersPerSecond());
                omega = UtilMethods.limitMagnitude(omega, drivetrain.getConfig().getClimbingMaxOmega());
            }

            // Limit movement when climbing
            if (faceClosestButton.get()) {
                godSubsystem.setAutoAligning(false);

                drivetrain.faceClosest(vx, vy, isFieldRelative);
            } else if (isPositionalRotation && godSubsystem.getCurrentMode() == Superstructure.CurrentMode.SCORING_MODE) {
                godSubsystem.setAutoAligning(false);

                double rotateX = -godSubsystem.getRotateX() * 10;
                double rotateY = godSubsystem.getRotateY() * 10;

                if (Math.abs(rotateX) > 1 || Math.abs(rotateY) > 1) {
                    drivetrain.faceDirection(vx, vy,
                            new Rotation2d(Math.atan2(rotateY, rotateX)).minus(Rotation2d.fromDegrees(90.0)),
                            isFieldRelative);
                } else {
                    drivetrain.move(vx, vy, 0, isFieldRelative);
                }
            } else if (trackTargetButton.get()) {
                godSubsystem.setAutoAligning(true);
                godSubsystem.handleAutoAlign(vx, vy, omega, isFieldRelative);
            } else {
                godSubsystem.setAutoAligning(false);
                drivetrain.move(vx, vy, omega, isFieldRelative);
//            drivetrain.move(0, 0, SmartDashboard.getNumber("Minimum omega command", 0.1), true);
            }
        }
    }

}