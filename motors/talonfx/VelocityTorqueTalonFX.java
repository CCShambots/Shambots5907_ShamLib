package frc.robot.ShamLib.motors.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class VelocityTorqueTalonFX extends EnhancedTalonFX {

  private double target; // In output units
  private double feedForward;

  /**
   * Constructor for a velocity configured TalonFX
   *
   * @param deviceNumber CAN ID
   * @param canbus name of the canbus (i.e. for CANivore)
   * @param gains PIDF gains
   * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output
   *     units
   */
  public VelocityTorqueTalonFX(
      int deviceNumber,
      String canbus,
      PIDSVGains gains,
      double feedForward,
      double inputToOutputRatio) {
    super(deviceNumber, canbus, inputToOutputRatio);

    this.feedForward = feedForward;

    TalonFXConfiguration config = new TalonFXConfiguration();

    configurePIDLoop(config.Slot0, gains);

    applyConfiguration(config);
  }

  /**
   * Constructor for a motion magic configured TalonFX
   *
   * @param deviceNumber CAN ID
   * @param gains PIDF gains
   * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output
   *     units
   */
  public VelocityTorqueTalonFX(
      int deviceNumber, PIDSVGains gains, double feedForward, double inputToOutputRatio) {
    this(deviceNumber, "", gains, feedForward, inputToOutputRatio);
  }

  /**
   * Set the target of the motor position
   *
   * @param target target position (in output units/sec)
   */
  public void setTarget(double target) {
    setControl(
        new VelocityTorqueCurrentFOC(outputToTicks(target))
            .withFeedForward(feedForward)
            .withSlot(0));

    this.target = target;
  }

  /**
   * Returns the target of the motor in output units
   *
   * @return output units
   */
  public double getTarget() {
    return target;
  }
}
