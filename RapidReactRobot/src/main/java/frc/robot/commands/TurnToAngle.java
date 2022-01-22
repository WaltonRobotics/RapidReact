package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.util.DebuggingLog;
import frc.robot.util.UtilMethods;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.SwerveDriveConfig.kMaxOmega;
//import static frc.robot.Robot.sDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

    private DoubleSupplier mTargetHeadingSupplier;
    private double mTargetHeading;
    private Drivetrain sDrivetrain;
    private ProfiledPIDController controller = new ProfiledPIDController
            (0.045, 0.0005, 0.000, new TrapezoidProfile.Constraints(kMaxOmega,2 ));

    public TurnToAngle(DoubleSupplier targetHeadingSupplier) {
        addRequirements(sDrivetrain);

        this.mTargetHeadingSupplier = targetHeadingSupplier;
    }

    public TurnToAngle(double targetHeading) {
        this(() -> targetHeading);
    }

    @Override
    public void initialize(){
        mTargetHeading = UtilMethods.restrictAngle(mTargetHeadingSupplier.getAsDouble(), -180.0, 180.0);
        controller.reset(new TrapezoidProfile.State(getHeading(),
                sDrivetrain.getAngularVelocityDegreesPerSec()));
        controller.setGoal(mTargetHeading);
    }
    @Override
    public void execute(){
        double turnRate = -controller.calculate(getHeading(),mTargetHeading);
        sDrivetrain.move(0,0,turnRate,true);
    }
    @Override
    public void end(boolean interrupted){
        sDrivetrain.move(0,0,0,true);
    }
    @Override
    public boolean isFinished() {
        return UtilMethods.isWithinTolerance(getHeading(),mTargetHeading,1.5);
    }
    private double getHeading() {
        return UtilMethods.restrictAngle(sDrivetrain.getHeading().getDegrees(),-180,180);
    }


}

