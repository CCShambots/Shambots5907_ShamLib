package frc.robot.ShamLib.Candle;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Timer;
import java.lang.reflect.Field;
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
  private RGB offRGB = new RGB(0, 0, 0);
  private Animation currentAnimation;

  private final int[] stateArray;

  // Timer used for deriving info about stuff
  Timer timer = new Timer();

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

    stateArray = new int[ledCount];

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = brightness;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    configAllSettings(configAll, 100);

    if (log) {
      timer.start();
    }
  }

  public CANdleEX(int canID, double brightness, int ledCount) {
    this(canID, brightness, ledCount, false);
  }

  public void periodic() {
    if (log) {
      Logger.recordOutput("CANdle/numLights", this.ledCount);

      switch (mode) {
        case ANIMATION:
          try {

            if (currentAnimation instanceof LarsonAnimation
                || currentAnimation instanceof StrobeAnimation) {
              Animation animation = currentAnimation;

              Class<?> superClass = animation.getClass().getSuperclass();

              Field speedField = superClass.getSuperclass().getDeclaredField("speed");
              Field ledOffsetField = superClass.getSuperclass().getDeclaredField("ledOffset");
              Field directionField = superClass.getDeclaredField("direction");
              Field sizeField = superClass.getDeclaredField("size");
              Field rField = superClass.getDeclaredField("r");
              Field gField = superClass.getDeclaredField("g");
              Field bField = superClass.getDeclaredField("b");

              speedField.setAccessible(true);
              ledOffsetField.setAccessible(true);
              directionField.setAccessible(true);
              sizeField.setAccessible(true);
              rField.setAccessible(true);
              gField.setAccessible(true);
              bField.setAccessible(true);

              double speed = (double) speedField.getDouble(animation);
              int ledOffset = (int) ledOffsetField.getInt(animation);
              // front, center, back
              int direction = (int) directionField.getInt(animation);
              int size = (int) sizeField.getInt(animation);
              int r = (int) rField.getInt(animation);
              int g = (int) gField.getInt(animation);
              int b = (int) bField.getInt(animation);

              RGB rgb = new RGB(r, g, b);

              if (currentAnimation instanceof LarsonAnimation) {

                // Estimate an arbitrary speed value
                double animationPeriod = 1 / speed;

                double halfAnimationPeriod = animationPeriod * .5;

                boolean goingForward = timer.get() % animationPeriod <= halfAnimationPeriod;

                int directionBasedOffset = 0;
                switch (direction) {
                  case 0:
                    directionBasedOffset = size;
                    break;
                  case 1:
                    directionBasedOffset = size / 2;
                    break;
                  default:
                    directionBasedOffset = 0;
                    break;
                }

                int ledsToTraverse = ledCount - ledOffset - directionBasedOffset;

                // Location of the start of the light pocket (assume going forward)
                int start =
                    (int)
                            ((timer.get() % halfAnimationPeriod)
                                / halfAnimationPeriod
                                * ledsToTraverse)
                        + ledOffset;

                // Flip this around if going backwards
                if (!goingForward) start = ledCount - start;

                for (int i = 0; i < start; i++) {
                  stateArray[i] = getIndexOfLoggedColor(offRGB);
                }

                for (int i = start; i < start + size; i++) {
                  stateArray[i] = getIndexOfLoggedColor(rgb);
                }

                for (int i = start + size; i < ledCount; i++) {
                  stateArray[i] = getIndexOfLoggedColor(offRGB);
                }

              } else if (currentAnimation instanceof StrobeAnimation) {

                // Estimate an arbitrary speed value
                double animationPeriod = speed * 30;

                double halfAnimationPeriod = animationPeriod * .5;

                boolean on = timer.get() % animationPeriod <= halfAnimationPeriod;

                RGB current = on ? rgb : offRGB;

                Arrays.fill(stateArray, getIndexOfLoggedColor(current));
              }
            } else if (currentAnimation instanceof RainbowAnimation) {

            }

          } catch (Exception e) {
            System.out.println("ANIMATION ERROR");
          }
          break;
        case SOLID:
          if (currentRGB != null) {
            int index = getIndexOfLoggedColor(currentRGB);

            Arrays.fill(stateArray, index);
          }

          break;
        case SEGS:
          if (currentSegs != null) {

            for (MultipleColorSegments.ColorSegmentInfo info : currentSegs.colorSegmentInfoList) {
              int infoIndex = getIndexOfLoggedColor(info.rgb);

              for (int i = info.startLED; i < info.startLED + info.numLEDs && i < ledCount; i++) {
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
    timer.restart();

    currentAnimation = animation;

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
