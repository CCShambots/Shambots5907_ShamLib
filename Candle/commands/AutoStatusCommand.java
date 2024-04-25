package frc.robot.ShamLib.Candle.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.Candle.MultipleColorSegments;
import frc.robot.ShamLib.Candle.RGB;
import frc.robot.ShamLib.Candle.RGBSegmentInfo;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class AutoStatusCommand extends Command {

  private final Consumer<MultipleColorSegments> setLEDs;

  private final RGB successRGB;
  private final RGB errorRGB;
  private final RGB offRGB;

  private final double blinkPeriod;

  private final int startOffset;
  private final int numLEDsPerSegment;
  private final int lastSegOverflow;

  private final BooleanSupplier[] conditions;

  private final Timer timer = new Timer();

  public AutoStatusCommand(
      Consumer<MultipleColorSegments> setLEDs,
      RGB successRGB,
      RGB errorRGB,
      RGB offRGB,
      int numLEDs,
      int startOffset,
      double blinkPeriod,
      BooleanSupplier... conditions) {
    this.setLEDs = setLEDs;
    this.successRGB = successRGB;
    this.errorRGB = errorRGB;
    this.offRGB = offRGB;
    this.conditions = conditions;
    this.startOffset = startOffset;

    this.blinkPeriod = blinkPeriod;

    this.numLEDsPerSegment = numLEDs / conditions.length;
    this.lastSegOverflow = numLEDs % conditions.length;
  }

  public AutoStatusCommand(
      Consumer<MultipleColorSegments> setLEDs,
      RGB successRGB,
      RGB errorRGB,
      int numLEDs,
      int startOffset,
      BooleanSupplier... conditions) {
    this(setLEDs, successRGB, errorRGB, new RGB(0, 0, 0), numLEDs, startOffset, 1, conditions);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    // Determine whether the lights should be flashing based on whether we're more than half way
    // through the blink cycle
    boolean flashOn = timer.get() % blinkPeriod > blinkPeriod / 2;

    MultipleColorSegments segments =
        new MultipleColorSegments(new RGBSegmentInfo(new RGB(0, 0, 0), startOffset));

    for (BooleanSupplier condition : conditions) {
      segments.addSegment(
          new RGBSegmentInfo(
              (condition.getAsBoolean() ? successRGB : (flashOn ? errorRGB : offRGB)),
              numLEDsPerSegment));
    }

    // Make sure to fill up all the LEDs by overflowing the last segment if necessary
    segments.addSegment(
        new RGBSegmentInfo(
            (conditions[conditions.length - 1].getAsBoolean()
                ? successRGB
                : (flashOn ? errorRGB : offRGB)),
            lastSegOverflow));

    setLEDs.accept(segments);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // No need for an isFinished(). The lights subsystem should handle entering and exiting this
  // command automatically
}
