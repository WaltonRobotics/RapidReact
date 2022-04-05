package frc.robot.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.averages.SimpleMovingAverage;

import static frc.robot.Constants.FieldConstants.kTargetHeightInches;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.RobotContainer.currentRobot;

public class LimelightHelper {

    private static final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTableEntry mTx = mTable.getEntry("tx");
    private static final NetworkTableEntry mTy = mTable.getEntry("ty");
    private static final NetworkTableEntry mTa = mTable.getEntry("ta");
    private static final NetworkTableEntry mTv = mTable.getEntry("tv");
    private static final NetworkTableEntry mCamtran = mTable.getEntry("camtran");
    private static final NetworkTableEntry mLedMode = mTable.getEntry("ledMode");
    private static final NetworkTableEntry mCamMode = mTable.getEntry("camMode");
    private static final NetworkTableEntry mPipeline = mTable.getEntry("pipeline");
    private static final NetworkTableEntry mSnapshot = mTable.getEntry("snapshot");
    private static final SimpleMovingAverage mTxMovingAverage = new SimpleMovingAverage(kTxWindowSize);
    private static final SimpleMovingAverage mTyMovingAverage = new SimpleMovingAverage(kTyWindowSize);
    private static boolean mIsLEDOn = false;
    private static int resetCount;

    private LimelightHelper() {

    }

    public static void updateData() {
        if (getTV() > 0) {
            mTxMovingAverage.addData(mTx.getDouble(0.0));
            mTyMovingAverage.addData(mTy.getDouble(0.0));
        }
        resetCount--;
        if (resetCount == 0) {
            resetSnapshot();
        }
    }

    /**
     * @return tx The x angle from target in degrees
     */
    public static double getTX() {
        return mTxMovingAverage.getMean();
    }

    /**
     * @return ty The y angle from target in degrees
     */
    public static double getTY() {
        return mTyMovingAverage.getMean();
    }

    /**
     * @return ta The area of the target
     */
    public static double getTA() {
        return mTa.getDouble(0);
    }

    /**
     * @return tv The number of targets in the field of view
     */
    public static double getTV() {
        return mTv.getDouble(0);
    }

    public static double[] getCamtran() {
        return mCamtran.getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    public static int getLEDMode() {
        return mLedMode.getNumber(0).intValue();
    }

    public static void setLEDMode(boolean on) {
        if (on) {
            mLedMode.setNumber(kLEDsOn);
        } else {
            mLedMode.setNumber(kLEDsOff);
        }

        NetworkTableInstance.getDefault().flush();

        mIsLEDOn = on;
    }

    public static int getCamMode() {
        return mCamMode.getNumber(0).intValue();
    }

    public static void setCamMode(boolean isVisionProcessing) {
        if (isVisionProcessing) {
            mCamMode.setNumber(kVisionMode);
        } else {
            mCamMode.setNumber(kDriverMode);
        }
    }

    public static int getPipeline() {
        return mPipeline.getNumber(0).intValue();
    }

    public static void setPipeline(int pipelineNumber) {
        mPipeline.setNumber(pipelineNumber);
    }

    public static void toggleLimelight() {
        mIsLEDOn = !mIsLEDOn;
        setLEDMode(mIsLEDOn);
    }

    /**
     * @return distance The distance to the target in meters
     */
    public static double getDistanceToTargetMeters() {
        return Units.feetToMeters(getDistanceToTargetFeet());
    }

    public static double getDistanceToTargetFeet() {
        return ((kTargetHeightInches - currentRobot.getShooterConfig().getLimelightMountingHeightInches()) /
                (Math.tan(Units.degreesToRadians(currentRobot.getShooterConfig().getLimelightMountingAngleDegrees() + getTY()))
                        * Math.cos(Units.degreesToRadians(getTX())))) / 12.0 + kLimelightOffsetFeet;
    }

    public static void takeSnapshot() {
        mSnapshot.setNumber(1);
        resetCount = 10;
    }

    public static void resetSnapshot() {
        mSnapshot.setNumber(0);
    }

}
