package frc.robot.ShamLib.motors.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedDashboardPIDSV implements LoggedDashboardInput {
  private static final String resetPath = "/reset";
  private static final String valuesOrder = "PIDSV";

  private final String key;
  private PIDSVGains defaultValue;
  private PIDSVGains value;
  private AtomicBoolean reset = new AtomicBoolean(false);

  private Consumer<PIDSVGains> onChange = null;

  private boolean sendToNT;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
          double[] numbers = value.toArray();

          if (sendToNT) {
            for (int i = 0; i < 5; i++) {
              SmartDashboard.putNumber(key + "/" + valuesOrder.charAt(i), numbers[i]);
            }
          }

          table.put(key + resetPath, reset.get());
        }

        @Override
        public void fromLog(LogTable table) {
          double[] numbers = new double[5];
          double[] defaults = defaultValue.toArray();

          for (int i = 0; i < 5; i++) {
            numbers[i] = SmartDashboard.getNumber(key + "/" + valuesOrder.charAt(i), defaults[i]);
          }

          value.set(numbers);

          reset.set(table.get(key + resetPath, reset.get()));
        }
      };

  public LoggedDashboardPIDSV(String key, PIDSVGains defaultValue, boolean sendToNT) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue.clone();
    this.sendToNT = sendToNT;

    if (sendToNT) {
      putNumbers(getNumbers());
      SmartDashboard.putBoolean(key + resetPath, reset.get());
    }

    periodic();

    Logger.registerDashboardInput(this);
  }

  public void setOnChange(Consumer<PIDSVGains> onChange) {
    this.onChange = onChange;
  }

  private void putNumbers(double[] numbers) {
    for (int i = 0; i < 5; i++) {
      SmartDashboard.putNumber(key + "/" + valuesOrder.charAt(i), numbers[i]);
    }
  }

  private double[] getNumbers() {
    double[] numbers = new double[5];
    double[] defaults = defaultValue.toArray();

    for (int i = 0; i < 5; i++) {
      numbers[i] = SmartDashboard.getNumber(key + "/" + valuesOrder.charAt(i), defaults[i]);
    }

    return numbers;
  }

  public void setDefault(PIDSVGains defaultValue) {
    this.defaultValue = defaultValue;
  }

  public PIDSVGains getDefault() {
    return defaultValue;
  }

  public void set(PIDSVGains value) {
    if (sendToNT) {
      putNumbers(value.toArray());
    }
  }

  public PIDSVGains get() {
    return value;
  }

  public boolean diff(PIDSVGains val) {
    return !get().equals(val);
  }

  @Override
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      if (reset.get()) {
        set(defaultValue);
        reset.set(false);

        SmartDashboard.putBoolean(key + resetPath, reset.get());
      }
      PIDSVGains dashboardVal = new PIDSVGains(getNumbers());

      reset.set(SmartDashboard.getBoolean(key + resetPath, reset.get()));

      if (diff(dashboardVal) && onChange != null) {
        value = dashboardVal;
        onChange.accept(value);
      }
    }
    Logger.processInputs(prefix, inputs);
  }
}
