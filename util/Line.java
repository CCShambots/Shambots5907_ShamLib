package frc.robot.ShamLib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Line {
  private final double m;
  private final double b;

  public Line(double m, double b) {
    this.m = m;
    this.b = b;
  }

  public static Line pointSlope(double m, Translation2d point) {
    double b = -m * point.getX() + point.getY();

    return new Line(m, b);
  }

  public static Line fromTwoPoints(Translation2d point1, Translation2d point2) {
    double m = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());

    return pointSlope(m, point2);
  }

  public static Line fromPose2d(Pose2d pose) {
    return Line.pointSlope(pose.getRotation().getTan(), pose.getTranslation());
  }

  public Line perpendicular(Translation2d atPoint) {
    return Line.pointSlope(-1 / m, atPoint);
  }

  public Translation2d getValue(double x) {
    return new Translation2d(x, m * x + b);
  }

  public Translation2d intersection(Line other) {
    if (other.m == m) {
      return new Translation2d(0, 0);
    } else {
      double xSolution = (other.b - b) / (m - other.m);

      return getValue(xSolution);
    }
  }

  public Translation2d closestIntersection(Translation2d currentPosition) {
    return intersection(perpendicular(currentPosition));
  }
}
