package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.*;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();

//    private final IndicatorLights lights = new IndicatorLights();

    private boolean isEnabled = false;


    public Drivetrain getDrivetrain() {
        return drivetrain;
    }


    public boolean isEnabled() {
        return isEnabled;
    }

    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }


    public double getCurrentTime() {
        return Timer.getFPGATimestamp();
    }






    public double getVx() {
        double forward = OI.forwardScale.apply(getForward());
        return forward * drivetrain.getConfig().getMaxSpeedMetersPerSecond();
    }

    public double getVy() {
        double strafe = OI.strafeScale.apply(getStrafe());
        return strafe * drivetrain.getConfig().getMaxSpeedMetersPerSecond();
    }

    public double getOmega() {
        double yaw = OI.yawScale.apply(getRotateX());

        double omega = 0;

        // Ensure at least the minimum turn omega is supplied to the drivetrain to prevent stalling
        if (Math.abs(getRotateX()) > yawScale.getDeadband()) {
            omega = Math.signum(yaw) * Math.max(Math.abs(yaw * drivetrain.getConfig().getMaxOmega()),
                    drivetrain.getConfig().getMinTurnOmega());
        }

        return omega;
    }

    public boolean isRobotMotionOverride() {
        return (Math.abs(getVx()) > 0.3
                || Math.abs(getVy()) > 0.3) && kMotionCorrectShooting;
    }



    public void updateShuffleboard(){
    }

    public AllianceColor getInferredAllianceColor() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return AllianceColor.RED;
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return AllianceColor.BLUE;
        } else {
            return allianceColorChooser.getSelected();
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

    @Override
    public void periodic() {
    }

    public enum AllianceColor {
        RED, BLUE
    }

}
