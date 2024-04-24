package frc.robot.ShamLib.Candle.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.Candle.RGB;
import java.util.ArrayList;
import java.util.List;

public class PulseSpeedUpCommand extends Command {

  private final CANdleEX candle;
  private final RGB onRGB;
  private final RGB offRGB;
  private final double totalTime; // The total time of the command
  private final double blinkTime; // The amount of time the lights will be on

  private final Timer timer = new Timer();
  private final List<Double> timeIndices = new ArrayList<>();

  private int currentTimeIndex = 0;
  private Mode currentMode = Mode.OFF;

  public PulseSpeedUpCommand(
      CANdleEX candle, RGB onRGB, RGB offRGB, double totalTime, double blinkTime, double startGap) {
    this.candle = candle;
    this.onRGB = onRGB;
    this.offRGB = offRGB;
    this.totalTime = totalTime;
    this.blinkTime = blinkTime;

    double runningTime = 0;
    double currentIncrement = startGap;

    while (runningTime <= totalTime) {
      runningTime += currentIncrement;
      timeIndices.add(runningTime);
      currentIncrement -= 0.1; // arbitrary speed up factor that looks like it might look good

      if (currentIncrement < .15) currentIncrement = .15;
    }
  }

  public PulseSpeedUpCommand(
      CANdleEX candle, RGB onRGB, double totalTime, double blinkTime, double startGap) {
    this(candle, onRGB, new RGB(0, 0, 0), totalTime, blinkTime, startGap);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    currentMode = Mode.OFF;
    currentTimeIndex = 0;
  }

  @Override
  public void execute() {
    if (currentMode == Mode.OFF) {
      if (timer.get() >= timeIndices.get(currentTimeIndex)) {
        currentMode = Mode.ON;
        candle.setLEDs(onRGB);
      }
    } else if (currentMode == Mode.ON) {
      if (timer.get() >= timeIndices.get(currentTimeIndex) + blinkTime) {
        currentMode = Mode.OFF;
        candle.setLEDs(offRGB);
        currentTimeIndex++;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    candle.setLEDs(onRGB);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= totalTime;
  }

  private enum Mode {
    OFF,
    ON
  }
}
