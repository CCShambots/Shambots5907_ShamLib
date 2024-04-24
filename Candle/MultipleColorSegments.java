package frc.robot.ShamLib.Candle;

import java.util.ArrayList;
import java.util.List;

public class MultipleColorSegments {
  final List<ColorSegmentInfo> colorSegmentInfoList = new ArrayList<>();

  private int offset;

  public MultipleColorSegments(int baseOffset, RGBSegmentInfo... segments) {
    offset = baseOffset;
    for (RGBSegmentInfo segment : segments) {
      colorSegmentInfoList.add(new ColorSegmentInfo(segment.rgb, segment.numLEDs, offset));
      offset += segment.numLEDs;
    }
  }

  public MultipleColorSegments(RGBSegmentInfo... segments) {
    this(0, segments);
  }

  public void addSegment(RGBSegmentInfo segment) {
    colorSegmentInfoList.add(new ColorSegmentInfo(segment.rgb, segment.numLEDs, offset));
    offset += segment.numLEDs;
  }

  static class ColorSegmentInfo {
    final RGB rgb;
    final int numLEDs;
    final int startLED;

    public ColorSegmentInfo(RGB rgb, int numLEDs, int startLED) {
      this.rgb = rgb;
      this.numLEDs = numLEDs;
      this.startLED = startLED;
    }
  }
}
