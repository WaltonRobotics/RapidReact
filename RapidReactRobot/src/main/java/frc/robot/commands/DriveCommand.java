package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();
    private static final double FORWARD_DEADBAND = 0.05;
    private static final double STRAFE_DEADBAND = 0.05;
    private static final double YAW_DEADBAND = 0.01;

    private static final double FORWARD_XPOSCALE = 0.6;
    private static final double STRAFE_XPOSCALE = 0.6;
    private static final double YAW_XPOSCALE = 0.75;


    public DriveCommand() {
        addRequirements(drivetrain);
    }




}
