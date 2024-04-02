package frc.robot.ShamLib.motors.talonfx;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class MotionMagicTalonFX extends EnhancedTalonFX {
  protected double target; // In output units

  private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  /**
   * Constructor for a motion magic configured TalonFX
   *
   * @param deviceNumber CAN ID
   * @param canbus name of the canbus (i.e. for CANivore)
   * @param gains PIDF gains
   * @param inputToOutputRatio number to multiply TalonFX rotations by to get output units
   * @param maxVel maximum velocity the motor should reach
   * @param maxAccel maximum acceleration the motor should undergo
   */
  public MotionMagicTalonFX(
      int deviceNumber,
      String canbus,
      PIDSVGains gains,
      double inputToOutputRatio,
      double maxVel,
      double maxAccel,
      double jerk) {
    super(deviceNumber, canbus, inputToOutputRatio);

    TalonFXConfiguration config = new TalonFXConfiguration();

    configurePIDLoop(config.Slot0, gains);

    // Set the acceleration and cruise velocity - see documentation
    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

    motionMagicConfigs.MotionMagicAcceleration = maxAccel;
    motionMagicConfigs.MotionMagicCruiseVelocity = maxVel;
    motionMagicConfigs.MotionMagicJerk = jerk;

    applyConfiguration(config);
  }

  public MotionMagicTalonFX(
      int deviceNumber,
      String canbus,
      PIDSVGains gains,
      double inputToOutputRatio,
      double maxVel,
      double maxAccel) {
    this(deviceNumber, canbus, gains, inputToOutputRatio, maxVel, maxAccel, 10000);
  }
  /**
   * Constructor for a motion magic configured TalonFX
   *
   * @param deviceNumber CAN ID
   * @param gains PIDF gains
   * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output
   *     units
   */
  public MotionMagicTalonFX(int deviceNumber, PIDSVGains gains, double inputToOutputRatio) {
    this(deviceNumber, "", gains, inputToOutputRatio, 0, 0);
  }

  /**
   * Constructor for a motion magic configured TalonFX
   *
   * @param deviceNumber CAN ID
   * @param gains PIDF gains
   * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output
   *     units
   * @param maxVel maximum velocity the motor should reach
   * @param maxAccel maximum acceleration the motor should underg
   */
  public MotionMagicTalonFX(
      int deviceNumber,
      PIDSVGains gains,
      double inputToOutputRatio,
      double maxVel,
      double maxAccel) {
    this(deviceNumber, "", gains, inputToOutputRatio, maxVel, maxAccel);
  }

  public MotionMagicTalonFX(
      int deviceNumber,
      PIDSVGains gains,
      double inputToOutputRatio,
      double maxVel,
      double maxAccel,
      double jerk) {
    this(deviceNumber, "", gains, inputToOutputRatio, maxVel, maxAccel, jerk);
  }

  /**
   * Constructor for a motion magic configured TalonFX
   *
   * @param deviceNumber CAN ID
   * @param canbus name of the canbus (i.e. for CANivore)
   * @param gains PIDF gains
   * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output
   *     units
   */
  public MotionMagicTalonFX(
      int deviceNumber, String canbus, PIDSVGains gains, double inputToOutputRatio) {
    this(deviceNumber, canbus, gains, inputToOutputRatio, 0, 0);
  }

  /**
   * Set the target of the motor position
   *
   * @param target target position (in output units)
   */
  public void setTarget(double target) {
    this.target = target;
    setControl(mmReq.withPosition(outputToTicks(target)).withSlot(0));
  }

  public void setFOC(boolean enabled) {
    mmReq.EnableFOC = enabled;
  }

  /**
   * Returns the target of the motor in output units
   *
   * @return output units
   */
  public double getTarget() {
    return target;
  }

  public void changeSpeed(double maxVelo, double maxAccel, double maxJerk) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    getConfigurator().refresh(config);

    config.MotionMagic.MotionMagicAcceleration = maxAccel;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelo;
    config.MotionMagic.MotionMagicJerk = maxJerk;

    getConfigurator().apply(config);
  }

  public void changeSpeed(double maxVelo, double maxAccel) {
    changeSpeed(maxVelo, maxAccel, 10000);
  }
}
