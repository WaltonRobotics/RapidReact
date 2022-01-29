package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake implements SubSubsystem {
    //following hardware have dummy channels
    private static final TalonFX mIntakeController = new TalonFX(1);
    private static final DoubleSolenoid intake1 =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,2);
    private static final DoubleSolenoid intake2 =
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM,3,4);

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
