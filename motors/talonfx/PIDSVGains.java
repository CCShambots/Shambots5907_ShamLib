package frc.robot.ShamLib.motors.talonfx;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Arrays;

public class PIDSVGains implements Sendable, Cloneable {
  private double[] gains = new double[5];

  public PIDSVGains(double kP, double kI, double kD, double kS, double kV) {
    gains = new double[] {kP, kI, kD, kS, kV};
  }

  public PIDSVGains(double[] values) {
    System.arraycopy(values, 0, gains, 0, values.length);
  }

  public PIDSVGains() {
    this(0, 0, 0, 0, 0);
  }

  public double getP() {
    return gains[0];
  }

  public double getI() {
    return gains[1];
  }

  public double getD() {
    return gains[2];
  }

  public double getS() {
    return gains[3];
  }

  public double getV() {
    return gains[4];
  }

  public double[] toArray() {
    return gains;
  }

  public void set(double[] values) {
    System.arraycopy(values, 0, gains, 0, values.length);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("P", this::getP, (value) -> gains[0] = value);
    builder.addDoubleProperty("I", this::getI, (value) -> gains[1] = value);
    builder.addDoubleProperty("D", this::getD, (value) -> gains[2] = value);
    builder.addDoubleProperty("S", this::getS, (value) -> gains[3] = value);
    builder.addDoubleProperty("V", this::getV, (value) -> gains[4] = value);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    PIDSVGains that = (PIDSVGains) o;

    for (int i = 0; i < this.gains.length; i++) {
      double diff = Math.abs(this.gains[i] - that.gains[i]);

      if (diff > 0.0000001) return false;
    }

    return true;
  }

  public PIDSVGains clone() {
    return new PIDSVGains(gains.clone());
  }

  @Override
  public int hashCode() {
    return Arrays.hashCode(gains);
  }
}
