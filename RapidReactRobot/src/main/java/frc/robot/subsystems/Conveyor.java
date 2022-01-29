package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Conveyor implements SubSubsystem {
    //front is intake to conveyor, back is conveyor to shooter
    private static final Spark mFrontConveyor = new Spark(1);    //dummy channel
    private static final Spark mBackConveyor = new Spark(2);     //dummy channel

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
