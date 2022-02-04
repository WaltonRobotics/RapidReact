package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Conveyor implements SubSubsystem {

    private final Spark transportController = new Spark(1);
    private final Spark feedController = new Spark(2);

    public Conveyor() {

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
