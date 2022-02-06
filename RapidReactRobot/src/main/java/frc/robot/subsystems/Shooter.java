package frc.robot.subsystems;

import frc.robot.vision.LimelightHelper;

public class Shooter implements SubSubsystem {

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {
        LimelightHelper.updateData();
    }

    @Override
    public void outputData() {

    }

}
