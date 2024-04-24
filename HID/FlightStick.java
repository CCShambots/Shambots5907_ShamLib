package frc.robot.ShamLib.HID;

import static frc.robot.ShamLib.HID.FlightStick.Button.*;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class FlightStick extends GenericHID {
  /** Represents a digital button on a Flight Stick. */
  public enum Button {
    Trigger(1),
    TopBase(2),
    TopLeft(3),
    TopRight(4),
    LeftTopLeft(5),
    LeftTopMiddle(6),
    LeftTopRight(7),
    LeftBottomRight(8),
    LeftBottomMiddle(9),
    LeftBottomLeft(10),
    RightTopRight(11),
    RightTopMiddle(12),
    RightTopLeft(13),
    RightBottomLeft(14),
    RightBottomMiddle(15),
    RightBottomRight(16);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and if not a Bumper button append `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {

      return this.name() + "Button";
    }
  }

  /** Represents an axis on a flight stick. */
  public enum Axis {
    LeftX(0),
    LeftY(1),
    LeftTrigger(2),
    RightTrigger(3);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      return this.name();
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public FlightStick(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_Joystick, port + 1);
  }

  /**
   * Get the X axis value of the controller.
   *
   * @return The axis value.
   */
  public double getXAxis() {
    return getRawAxis(Axis.LeftX.value);
  }

  /**
   * Get the Y axis value of the controller.
   *
   * @return The axis value.
   */
  public double getYAxis() {
    return getRawAxis(Axis.LeftY.value);
  }

  /**
   * Get the twist axist value of the controller
   *
   * @return The axis value.
   */
  public double getTwistAxis() {
    return getRawAxis(Axis.LeftTrigger.value);
  }

  /**
   * Get the throttle axis value of the controller (bounded [0, 1]).
   *
   * @return The axis value.
   */
  public double getThrottleAxis() {
    return getRawAxis(Axis.RightTrigger.value);
  }

  /**
   * Read the value of the trigger button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTrigger() {
    return getRawButton(Trigger.value);
  }

  /**
   * Whether the left bumper (LB) was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTriggerPressed() {
    return getRawButtonPressed(Trigger.value);
  }

  /**
   * Constructs an event instance around the trigger's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the right bumper's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent trigger(EventLoop loop) {
    return new BooleanEvent(loop, this::getTrigger);
  }

  /**
   * Whether the Trigger was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTriggerReleased() {
    return getRawButtonReleased(Trigger.value);
  }

  /**
   * Read the value of the top base button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTopBase() {
    return getRawButton(TopBase.value);
  }

  /**
   * Read the value of the top left button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTopLeft() {
    return getRawButton(TopLeft.value);
  }

  /**
   * Read the value of the top right button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTopRight() {
    return getRawButton(TopRight.value);
  }

  /**
   * Whether the top base was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTopBasePressed() {
    return getRawButtonPressed(TopBase.value);
  }

  /**
   * Whether the top left was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTopLeftPressed() {
    return getRawButtonPressed(TopLeft.value);
  }

  /**
   * Whether the top right was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTopRightPressed() {
    return getRawButtonPressed(TopRight.value);
  }

  /**
   * Whether the top base was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTopBaseReleased() {
    return getRawButtonReleased(TopBase.value);
  }

  /**
   * Whether the top left was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTopLeftReleased() {
    return getRawButtonReleased(TopLeft.value);
  }

  /**
   * Whether the top right was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTopRightReleased() {
    return getRawButtonReleased(TopRight.value);
  }

  /**
   * Constructs an event instance around the top base's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left bumper's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent topBase(EventLoop loop) {
    return new BooleanEvent(loop, this::getTopBase);
  }

  /**
   * Constructs an event instance around the top left's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left bumper's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent topLeft(EventLoop loop) {
    return new BooleanEvent(loop, this::getTopLeft);
  }

  /**
   * Constructs an event instance around the top right's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left bumper's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent topRight(EventLoop loop) {
    return new BooleanEvent(loop, this::getTopRight);
  }

  /**
   * Constructs an event instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This
   *     value should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
    return new BooleanEvent(loop, () -> getTwistAxis() > threshold);
  }

  /**
   * Constructs an event instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent leftTrigger(EventLoop loop) {
    return leftTrigger(0.5, loop);
  }

  /**
   * Constructs an event instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This
   *     value should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
    return new BooleanEvent(loop, () -> getThrottleAxis() > threshold);
  }

  /**
   * Constructs an event instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public BooleanEvent rightTrigger(EventLoop loop) {
    return rightTrigger(0.5, loop);
  }
}
