package frc.lib.strykeforce.thirdcoast.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Applies exponential scaling and deadband to joystick inputs */
public class ExpoScale implements Sendable {

  private double deadband;
  private double scale;

  public ExpoScale(double deadband, double scale) {
    this.deadband = deadband;
    this.scale = scale;
  }

  /**
   * Return the joystick axis position input adjusted on an exponential scale with deadband
   * adjustment.
   *
   * @param input the joystick axis position
   * @return the adjusted input value, range is -1.0 to 1.0
   */
  public double apply(double input) {
    double offset = 1.0 / (scale * Math.pow(1 - deadband, 3) + (1 - scale) * (1 - deadband));

    double y;

    if (Math.abs(input) < deadband) {
      return 0;
    }

    y = input > 0 ? input - deadband : input + deadband;
    return (scale * Math.pow(y, 3) + (1 - scale) * y) * offset;
  }

  public double getDeadband() {
    return deadband;
  }

  public void setDeadband(double deadband) {
    this.deadband = deadband;
  }

  public double getScale() {
    return scale;
  }

  public void setScale(double scale) {
    this.scale = scale;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ExpoScale");
    builder.addDoubleProperty("deadband", this::getDeadband, this::setDeadband);
    builder.addDoubleProperty("scale", this::getScale, this::setScale);
  }
}
