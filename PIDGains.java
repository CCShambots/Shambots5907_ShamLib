package frc.robot.ShamLib;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;

public class PIDGains {
  public final double p;
  public final double i;
  public final double d;

  public PIDGains(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
  }

  public PIDGains() {
    this(0, 0, 0);
  }

public PIDController applyToController() {
    return new PIDController(p, i, d);
  }

  public PIDConstants toPIDConstants() {
    return new PIDConstants(this.p, this.i, this.d);
  }
}
