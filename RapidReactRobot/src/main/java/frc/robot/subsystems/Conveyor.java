package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Conveyor implements SubSubsystem {

    //front is intake to conveyor, back is conveyor to shooter
    private static final Spark frontConveyorController = new Spark(1);    //dummy channel
    private static final Spark backConveyorController = new Spark(2);     //dummy channel

    public Conveyor(){
    }

    public void setFrontDutyCycle(double targetDutyCycle) {
        frontConveyorController.setVoltage(targetDutyCycle);
    }

    public void setBackDutyCycle(double targetDutyCycle) {
        backConveyorController.setVoltage(targetDutyCycle);
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
