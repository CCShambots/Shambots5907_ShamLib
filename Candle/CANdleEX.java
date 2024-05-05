package frc.robot.ShamLib.Candle;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class CANdleEX extends CANdle {

  private final int ledCount;

  private final boolean log;

  private final List<String> usedColors = new ArrayList<>();

  private Mode mode = Mode.SOLID;
  private MultipleColorSegments currentSegs = new MultipleColorSegments();
  private RGB currentRGB = new RGB(0, 0, 0);

  /**
   * Constructs a new CANdleEX object
   *
   * @param canID id of the candle
   * @param brightness brightness [0.0-1.0]
   * @param ledCount number of leds in the string
   * @param log whether to log data
   */
  public CANdleEX(int canID, double brightness, int ledCount, boolean log) {
    super(canID);

    this.ledCount = ledCount;

    this.log = log;

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = brightness;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    configAllSettings(configAll, 100);
  }

  public CANdleEX(int canID, double brightness, int ledCount) {
    this(canID, brightness, ledCount, false);
  }

  public void periodic() {
    if (log) {
      Logger.recordOutput("CANdle/numLights", this.ledCount);

      int[] stateArray = new int[ledCount];

      switch (mode) {
          // TODO: Figure out animation stuff
        case ANIMATION:
          break;
        case SOLID:
          if (currentRGB != null) {
            int index = getIndexOfLoggedColor(currentRGB);

            Arrays.fill(stateArray, index);
          }

          break;
        case SEGS:
          if (currentSegs != null) {
            int currentLED = 0;

            for (MultipleColorSegments.ColorSegmentInfo info : currentSegs.colorSegmentInfoList) {
              int infoIndex = getIndexOfLoggedColor(info.rgb);

              for (int i = info.startLED; i < info.startLED + info.numLEDs && i < ledCount; i++) {
                currentLED++;
                stateArray[i] = infoIndex;
              }
            }
          }

          break;
        default:
          break;
      }

      Logger.recordOutput("CANdle/states", stateArray);
      Logger.recordOutput("CANdle/colors", this.usedColors.stream().toArray(String[]::new));
    }
  }

  /**
   * Gets (or generates) the index of the passed color in the usedColors array (for logging)
   *
   * @return index of color
   */
  public int getIndexOfLoggedColor(RGB rgb) {
    int index = usedColors.indexOf(rgb.toLoggable());

    if (index == -1) {
      usedColors.add(rgb.toLoggable());
      index = usedColors.size() - 1;
    }

    return index;
  }

  public int getLedCount() {
    return ledCount;
  }

  public void setBrightness(double percent) {
    configBrightnessScalar(percent, 0);
  }

  @Override
  public ErrorCode animate(Animation animation, int animSlot) {
    mode = Mode.ANIMATION;
    return super.animate(animation, animSlot);
  }

  public void setLEDs(RGB values) {
    mode = Mode.SOLID;
    currentRGB = values;
    clearAnimation(0);
    setLEDs(values.R, values.G, values.B);
  }

  public void setLEDs(MultipleColorSegments segs) {
    mode = Mode.SEGS;
    currentSegs = segs;

    clearAnimation(0);
    for (MultipleColorSegments.ColorSegmentInfo info : segs.colorSegmentInfoList) {
      RGB rgb = info.rgb;
      setLEDs(rgb.R, rgb.G, rgb.B, 0, info.startLED, info.numLEDs);
    }
  }

  public enum Mode {
    ANIMATION,
    SOLID,
    SEGS
  }
}
