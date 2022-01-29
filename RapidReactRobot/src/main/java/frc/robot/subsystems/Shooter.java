package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter implements SubSubsystem {
    private final TalonFX mFlywheelMaster = new TalonFX(1);
    private final TalonFX mFlywheelSlave = new TalonFX(2);
    private final Solenoid mAdjustableHoodSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

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
