package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SmartDashboardKeys.*;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Intake getIntake() {
        return intake;
    }

    public Conveyor getConveyor() {
        return conveyor;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Climber getClimber() {
        return climber;
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(kIntakePeriodicIOKey, intake.getPeriodicIOSendable());
        SmartDashboard.putData(kConveyorPeriodicIOKey, conveyor.getPeriodicIOSendable());
        SmartDashboard.putData(kShooterPeriodicIOKey, shooter.getPeriodicIOSendable());
        SmartDashboard.putData(kClimberPeriodicIOKey, climber.getPeriodicIOSendable());
    }

}
