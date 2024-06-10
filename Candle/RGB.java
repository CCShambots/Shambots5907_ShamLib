package frc.robot.ShamLib.Candle;

public class RGB {
  public final int R;
  public final int G;
  public final int B;

  public RGB(int r, int g, int b) {
    R = r;
    G = g;
    B = b;
  }

  public String toLoggable() {
    return R + "," + G + "," + B;
  }
}
