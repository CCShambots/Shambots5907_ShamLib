package frc.robot.ShamLib.Candle.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.Candle.MultipleColorSegments;
import frc.robot.ShamLib.Candle.RGB;
import frc.robot.ShamLib.Candle.RGBSegmentInfo;

import java.util.function.Consumer;

public class TimedColorFlowCommand extends Command {

  private final Timer timer = new Timer();
  private final int numLEDS;
  private final int startOffset;
  private final Consumer<MultipleColorSegments> setLEDs;
  private final double totalSeconds;
  private final RGB color;
  private final RGB backgroundColor;

  /**
   * 
   * @param numLEDS Number of LEDs (Do NOT include lights in the start offset)
   * @param startOffset Number of LEDs to offset by ()
   * @param setLEDs Consumer that will set the light values on the CANdle
   * @param totalSeconds Total time the animation will run
   * @param color The color to fill with
   * @param backgroundColor The backgroudn color that will be there initially
   */
  public TimedColorFlowCommand(
      int numLEDS,
      int startOffset,
      Consumer<MultipleColorSegments> setLEDs,
      double totalSeconds,
      RGB color,
      RGB backgroundColor) {
    this.numLEDS = numLEDS;
    this.startOffset = startOffset;
    this.setLEDs = setLEDs;
    this.totalSeconds = totalSeconds;
    this.color = color;
    this.backgroundColor = backgroundColor;
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    double amountElapsed = timer.get() / totalSeconds;

    int numLightsIlluminated = (int) (amountElapsed * numLEDS);

    MultipleColorSegments segs =
        new MultipleColorSegments(
            new RGBSegmentInfo(new RGB(0, 0, 0), startOffset),
            new RGBSegmentInfo(color, numLightsIlluminated),
            new RGBSegmentInfo(backgroundColor, numLEDS - numLightsIlluminated));

    setLEDs.accept(segs);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(totalSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
