package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

public class PVApriltagCam {
  private final PVApriltagIO io;
  private final PVApriltagInputsAutoLogged inputs = new PVApriltagInputsAutoLogged();
  private final String name;
  private final AprilTagFieldLayout fieldLayout;

  public PVApriltagCam(
      String name,
      ShamLibConstants.BuildMode buildMode,
      Transform3d botToCam,
      AprilTagFieldLayout fieldLayout) {
    io = getNewIO(buildMode, name, botToCam, fieldLayout);
    this.name = name;
    this.fieldLayout = fieldLayout;
  }

  public void setPoseEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    io.setEstimationStrategy(strategy);
  }

  public void setMultiTagFallbackEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    io.setMultiTagFallbackStrategy(strategy);
  }

  public void setReferencePose(Pose2d pose) {
    io.setReferencePose(pose);
  }

  public void setLastPose(Pose2d pose) {
    io.setLastPose(pose);
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  private Matrix<N3, N1> getXYThetaStdDev(Pose2d pose, int[] tagIds) {
    double totalDistance = 0.0;
    int nonErrorTags = 0;

    // make the std dev greater based on how far away the tags are (trust estimates from further
    // tags less)
    // algorithm from frc6328

    for (int tagId : tagIds) {
      var tagOnField = fieldLayout.getTagPose(tagId);

      if (tagOnField.isPresent()) {
        totalDistance +=
            pose.getTranslation().getDistance(tagOnField.get().toPose2d().getTranslation());
        nonErrorTags++;
      }
    }
    double avgDistance = totalDistance / nonErrorTags;

    double xyStdDev = Math.pow(avgDistance, 2.0) / nonErrorTags;
    double thetaStdDev = Math.pow(avgDistance, 2.0) / nonErrorTags;

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public TimestampedPoseEstimator.TimestampedVisionUpdate getLatestEstimate() {
    return new TimestampedPoseEstimator.TimestampedVisionUpdate(
        inputs.timestamp,
        inputs.poseEstimate,
        getXYThetaStdDev(inputs.poseEstimate, inputs.targetsUsed));
  }

  private PVApriltagIO getNewIO(
      ShamLibConstants.BuildMode buildMode,
      String name,
      Transform3d botToCam,
      AprilTagFieldLayout fieldLayout) {
    return switch (buildMode) {
      case REPLAY -> new PVApriltagIO() {};
      default -> new PVApriltagIOReal(name, botToCam, fieldLayout);
    };
  }
}
