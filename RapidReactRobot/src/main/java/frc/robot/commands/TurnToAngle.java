package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.UtilMethods;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveDriveConfig.kMaxOmega;
import static frc.robot.RobotContainer.godSubsystem;
//import static frc.robot.Robot.sDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private final DoubleSupplier targetHeadingSupplier;
    private double targetHeading;

    private final ProfiledPIDController controller = new ProfiledPIDController
            (0.045, 0.0005, 0.000, new TrapezoidProfile.Constraints(kMaxOmega,2 ));

    public TurnToAngle(DoubleSupplier targetHeadingSupplier) {
        addRequirements(drivetrain);

        this.targetHeadingSupplier = targetHeadingSupplier;

        SmartDashboard.putData("Turn to angle controller", controller);
    }

    public TurnToAngle(double targetHeading) {
        this(() -> targetHeading);
    }

    @Override
    public void initialize(){
        targetHeading = UtilMethods.restrictAngle(targetHeadingSupplier.getAsDouble(), -180.0, 180.0);

        controller.reset(new TrapezoidProfile.State(getHeading(),
                drivetrain.getAngularVelocityDegreesPerSec()));

        controller.setGoal(targetHeading);
    }

    @Override
    public void execute(){
        double turnRate = -controller.calculate(getHeading(), targetHeading);

        drivetrain.move(0,0, turnRate,false);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.move(0,0,0,false);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    private double getHeading() {
        return UtilMethods.restrictAngle(drivetrain.getHeading().getDegrees(),-180,180);
    }

}

