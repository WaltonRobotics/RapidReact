package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake implements SubSubsystem {

    private final Spark leftIntakeController = new Spark(1);
    private final Spark rightIntakeController = new Spark(2);

    private final Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private final Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);


    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {

    }

    @Override
    public void outputData() {

    }

}
