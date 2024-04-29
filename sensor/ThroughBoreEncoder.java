package frc.robot.ShamLib.sensor;

import static java.lang.Math.PI;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class ThroughBoreEncoder {
  private final DutyCycle cycle;
  private final double offset;
  private boolean invert = false;

  /**
   * REV Robotics through bore encoder wrappper.
   * WARNING: DO NOT trust the values this returns instantly after it is instantiated.
   * Because it is a digital duty cycle, it takes a non-zero amount of time for the reading to stabilize to the correct value. 
   * @param port Digital port of the encoder
   * @param offsetDegrees Degrees to offset the encoder reading by. Should be determined by using the value the encoder reads when at the mechanisms "zero" position
   */
  public ThroughBoreEncoder(int port, double offsetDegrees) {
    this.cycle = new DutyCycle(new DigitalInput(port));
    this.offset = offsetDegrees / 360.0;
  }

  /**
   * Enable or disable inversion on the encoder
   * @param value true or false
   */
  public void setInverted(boolean value) {
    invert = value;
  }

  public double getRaw() {
    return Math.IEEEremainder(((invert ? -1 : 1) * cycle.getOutput()) - offset, 1);
  }

  public double getDegrees() {
    return getRaw() * 360;
  }

  public double getRadians() {
    return getRaw() * PI * 2;
  }
}
