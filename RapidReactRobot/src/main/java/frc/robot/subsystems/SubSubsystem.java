package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;

public interface SubSubsystem {

    void zeroSensors();

    void collectData();

    void outputData();

    Sendable getPeriodicIOSendable();

}
