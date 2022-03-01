package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.DriverPreferences.kMaxTranslationalAccelerationMsecSquared;
import static frc.robot.OI.driveGamepad;
import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final SlewRateLimiter vxRateLimiter;
    private final SlewRateLimiter vyRateLimiter;

    private static boolean enabled = true;

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
            double forward = OI.forwardScale.apply(getForward());
            double strafe = OI.strafeScale.apply(getStrafe());
            double yaw = OI.yawScale.apply(getYaw());
            double vx = vxRateLimiter.calculate(forward * drivetrain.getConfig().getMaxSpeedMetersPerSecond());
            double vy = vyRateLimiter.calculate(strafe * drivetrain.getConfig().getMaxSpeedMetersPerSecond());
            double omega = yaw * drivetrain.getConfig().getMaxOmega();

            // Limit movement when climbing
            if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.CLIMBING_MODE) {
                vx = UtilMethods.limitMagnitude(vx, drivetrain.getConfig().getClimbingMaxMetersPerSecond());
                vy = UtilMethods.limitMagnitude(vy, drivetrain.getConfig().getClimbingMaxMetersPerSecond());
                omega = UtilMethods.limitMagnitude(omega, drivetrain.getConfig().getClimbingMaxOmega());
            }

            drivetrain.move(vx, vy, omega, true);
        }
    }

    public double getForward() {
        return -driveGamepad.getLeftY();
    }

    public double getStrafe() {
        return -driveGamepad.getLeftX();
    }

    public double getYaw() {
        return -driveGamepad.getRightX();
    }

}
