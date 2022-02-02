package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake implements SubSubsystem {
    public boolean kIsIntakeControllerInverted;
    public double kIntakeDutyCycle;
    public double kOuttakeDutyCycle;
    public double kSettleTime;

    private final Spark mIntakeController = new Spark(9);//kIntakeID dummy value
    //private final Solenoid mDeploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1); //deployIntakeSolenoid ID

    public Intake() {
        mIntakeController.setInverted(true);
    }

//    public void setDeployed(boolean isDeployed) {
//        mDeploySolenoid.set(isDeployed);
//    }

    public void setVoltage(double voltage) {
        mIntakeController.setVoltage(voltage);
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