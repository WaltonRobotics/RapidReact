package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.IntakeConfig;
import frc.robot.util.EnhancedBoolean;

import static frc.robot.Constants.Intake.rollUpTimeoutSeconds;
import static frc.robot.RobotContainer.currentRobot;

public class Intake implements SubSubsystem {

    private final IntakeConfig config = currentRobot.getIntakeConfig();

    private final VictorSPX leftIntakeController = new VictorSPX(config.getLeftIntakeControllerConfig().getChannelOrID());
    private final VictorSPX rightIntakeController = new VictorSPX(config.getRightIntakeControllerConfig().getChannelOrID());

    private final Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            config.getLeftIntakeSolenoidChannel());

    private final Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            config.getRightIntakeSolenoidChannel());

    private final PeriodicIO periodicIO = new PeriodicIO();

    public Intake() {
        leftIntakeController.setInverted(config.getLeftIntakeControllerConfig().isInverted());
        rightIntakeController.setInverted(config.getRightIntakeControllerConfig().isInverted());

        leftIntakeController.configVoltageCompSaturation(12.0);
        leftIntakeController.enableVoltageCompensation(true);

        rightIntakeController.configVoltageCompSaturation(12.0);
        rightIntakeController.enableVoltageCompensation(true);

        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_1_General, 200);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 200);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        leftIntakeController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);

        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_1_General, 200);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 200);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        rightIntakeController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {
        double currentTime = Timer.getFPGATimestamp();

        leftIntakeSolenoid.set(periodicIO.leftIntakeDeployDemand);
        rightIntakeSolenoid.set(periodicIO.rightIntakeDeployDemand);

        boolean leftIntakeNeedsToRollUp = periodicIO.leftIntakeDeployBoolean.isFallingEdge();
        boolean rightIntakeNeedsToRollUp = periodicIO.rightIntakeDeployBoolean.isFallingEdge();

        if (leftIntakeNeedsToRollUp) {
            periodicIO.leftIntakeRollUpTimeout = currentTime + rollUpTimeoutSeconds;
        }

        if (rightIntakeNeedsToRollUp) {
            periodicIO.rightIntakeRollUpTimeout = currentTime + rollUpTimeoutSeconds;
        }
    }

    @Override
    public void outputData() {
        switch (periodicIO.intakeControlState) {
            case OPEN_LOOP:
                if (isLeftIntakeRollUpNeeded()) {
                    leftIntakeController.set(VictorSPXControlMode.PercentOutput, config.getLeftOuttakePercentOutput());
                } else {
                    leftIntakeController.set(VictorSPXControlMode.PercentOutput, periodicIO.leftIntakeDemand);
                }

                if (isRightIntakeRollUpNeeded()) {
                    rightIntakeController.set(VictorSPXControlMode.PercentOutput, config.getRightOuttakePercentOutput());
                } else {
                    rightIntakeController.set(VictorSPXControlMode.PercentOutput, periodicIO.rightIntakeDemand);
                }
                break;
            case DISABLED:
                if (isLeftIntakeRollUpNeeded()) {
                    leftIntakeController.set(VictorSPXControlMode.PercentOutput, config.getLeftOuttakePercentOutput());
                } else {
                    leftIntakeController.set(VictorSPXControlMode.PercentOutput, 0.0);
                }

                if (isRightIntakeRollUpNeeded()) {
                    rightIntakeController.set(VictorSPXControlMode.PercentOutput, config.getRightOuttakePercentOutput());
                } else {
                    rightIntakeController.set(VictorSPXControlMode.PercentOutput, 0.0);
                }
                break;
        }

        periodicIO.leftIntakeDeployBoolean.set(periodicIO.leftIntakeDeployDemand);
        periodicIO.rightIntakeDeployBoolean.set(periodicIO.rightIntakeDeployDemand);
    }

    @Override
    public Sendable getPeriodicIOSendable() {
        return periodicIO;
    }

    private boolean isLeftIntakeRollUpNeeded() {
        return !periodicIO.leftIntakeDeployDemand && Timer.getFPGATimestamp() < periodicIO.leftIntakeRollUpTimeout;
    }

    private boolean isRightIntakeRollUpNeeded() {
        return !periodicIO.rightIntakeDeployDemand && Timer.getFPGATimestamp() < periodicIO.rightIntakeRollUpTimeout;
    }

    public IntakeControlState getIntakeControlState() {
        return periodicIO.intakeControlState;
    }

    public void setIntakeControlState(IntakeControlState intakeControlState) {
        periodicIO.intakeControlState = intakeControlState;
    }

    public double getLeftIntakeDemand() {
        return periodicIO.leftIntakeDemand;
    }

    public void setLeftIntakeDemand(double leftIntakeDemand) {
        periodicIO.leftIntakeDemand = leftIntakeDemand;
    }

    public double getRightIntakeDemand() {
        return periodicIO.rightIntakeDemand;
    }

    public void setRightIntakeDemand(double rightIntakeDemand) {
        periodicIO.rightIntakeDemand = rightIntakeDemand;
    }

    public boolean isLeftIntakeDeployed() {
        return periodicIO.leftIntakeDeployDemand;
    }

    public void setLeftIntakeDeployStateDemand(boolean leftIntakeDeployStateDemand) {
        periodicIO.leftIntakeDeployDemand = leftIntakeDeployStateDemand;
    }

    public void toggleLeftIntakeDeployStateDemand() {
        setLeftIntakeDeployStateDemand(!isLeftIntakeDeployed());
    }

    public boolean isRightIntakeDeployed() {
        return periodicIO.rightIntakeDeployDemand;
    }

    public void setRightIntakeDeployStateDemand(boolean rightIntakeDeployStateDemand) {
        periodicIO.rightIntakeDeployDemand = rightIntakeDeployStateDemand;
    }

    public void toggleRightIntakeDeployStateDemand() {
        setRightIntakeDeployStateDemand(!isRightIntakeDeployed());
    }

    public IntakeConfig getConfig() {
        return config;
    }

    public enum IntakeControlState {
        OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO implements Sendable {
        // Outputs
        public double leftIntakeDemand;
        public double rightIntakeDemand;
        public boolean leftIntakeDeployDemand;
        public EnhancedBoolean leftIntakeDeployBoolean = new EnhancedBoolean();
        public boolean rightIntakeDeployDemand;
        public EnhancedBoolean rightIntakeDeployBoolean = new EnhancedBoolean();
        public IntakeControlState intakeControlState = IntakeControlState.DISABLED;
        public double leftIntakeRollUpTimeout;
        public double rightIntakeRollUpTimeout;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Intake Control State", () -> intakeControlState.name(), (x) -> {
            });
            builder.addDoubleProperty("Left Intake Demand", () -> leftIntakeDemand, (x) -> {
            });
            builder.addDoubleProperty("Right Intake Demand", () -> rightIntakeDemand, (x) -> {
            });
            builder.addBooleanProperty("Left Intake Deploy State Demand", () -> leftIntakeDeployDemand, (x) -> {
            });
            builder.addBooleanProperty("Right Intake Deploy State Demand", () -> rightIntakeDeployDemand, (x) -> {
            });
        }
    }
}