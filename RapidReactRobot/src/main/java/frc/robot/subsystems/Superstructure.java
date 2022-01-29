package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();

    private static Superstructure superstructure;


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

}
