package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake implements SubSubsystem {
    //following hardware have dummy channels
    private static final Spark intakeController = new Spark(1);
    private static final DoubleSolenoid intake1 =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,2);
    private static final DoubleSolenoid intake2 =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM,3,4);

    public Intake(){
    }

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
