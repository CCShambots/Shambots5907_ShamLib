package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PVApriltagCam {
  private final PVApriltagIO io;
  private final PVApriltagIO.PVApriltagInputs inputs = new PVApriltagIO.PVApriltagInputs();
  private final String name;
  private final AprilTagFieldLayout fieldLayout;
  private final double trustCutOff;
  private double tagDistancePower = 0.33;
  private double tagDistanceScalar = 1;
  private final PhotonPoseEstimator photonPoseEstimator;

  private UnaryOperator<PhotonPipelineResult> preprocess = null;
  private Function<EstimatedRobotPose, TimestampedPoseEstimator.TimestampedVisionUpdate>
      postprocess = this::defaultPostProcess;

  /**
   * Constructor for a Photon Vision camera that can track AprilTags
   *
   * @param name the name of the camera
   * @param buildMode The current build mode (to adjust IO settings for replay and sim moments)
   * @param botToCam The transform that provides the camera's offset relative to the bot
   * @param fieldLayout The layout of Apriltags on the field
   * @param trustCutOff Distance (meters) where trust should be scaled back radically (because far
   *     tags become highly inaccurate)
   */
  public PVApriltagCam(
      String name,
      ShamLibConstants.BuildMode buildMode,
      Transform3d botToCam,
      AprilTagFieldLayout fieldLayout,
      double trustCutOff) {
    io = getNewIO(buildMode, name);

    // Default to LOWEST_AMBIGUITY
    // In most cases the user should probably configure this to use multi tag
    photonPoseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, botToCam);

    this.name = name;
    this.fieldLayout = fieldLayout;
    this.trustCutOff = trustCutOff;
  }

  public void setBotToCamTransform(Transform3d transform) {
    photonPoseEstimator.setRobotToCameraTransform(transform);
  }

  public TimestampedPoseEstimator.TimestampedVisionUpdate defaultPostProcess(
      EstimatedRobotPose estimate) {
    return new TimestampedPoseEstimator.TimestampedVisionUpdate(
        estimate.timestampSeconds, estimate.estimatedPose.toPose2d(), getXYThetaStdDev(estimate));
  }

  public void setPreProcess(UnaryOperator<PhotonPipelineResult> preprocess) {
    this.preprocess = preprocess;
  }

  public void setPostProcess(
      Function<EstimatedRobotPose, TimestampedPoseEstimator.TimestampedVisionUpdate> postprocess) {
    this.postprocess = postprocess;
  }

  public void setPoseEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    photonPoseEstimator.setPrimaryStrategy(strategy);
  }

  public void setMultiTagFallbackEstimationStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    photonPoseEstimator.setMultiTagFallbackStrategy(strategy);
  }

  public void setReferencePose(Pose2d pose) {
    photonPoseEstimator.setReferencePose(pose);
  }

  public String getName() {
    return name;
  }

  public void setLastPose(Pose2d pose) {
    photonPoseEstimator.setLastPose(pose);
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    Logger.recordOutput("Vision/" + name + "/tags", getCurrentVisionTagPoses());
  }

  public Pose3d[] getCurrentVisionTagPoses() {
    return inputs.frame.targets.stream()
        .map(e -> fieldLayout.getTagPose(e.getFiducialId()).orElseGet(() -> new Pose3d()))
        .toArray(Pose3d[]::new);
  }

  public void setTagDistancePower(double val) {
    tagDistancePower = val;
  }

  public void setTagDistanceScalar(double val) {
    tagDistanceScalar = val;
  }

  public Matrix<N3, N1> getXYThetaStdDev(EstimatedRobotPose pose) {
    double totalDistance = 0.0;
    int nonErrorTags = 0;

    // make the std dev greater based on how far away the tags are (trust estimates from further
    // tags less)
    // algorithm from frc6328 - Mechanical Advantage my beloved

    for (var tag : pose.targetsUsed) {
      var tagOnField = fieldLayout.getTagPose(tag.getFiducialId());

      if (tagOnField.isPresent()) {
        totalDistance +=
            pose.estimatedPose
                .toPose2d()
                .getTranslation()
                .getDistance(tagOnField.get().toPose2d().getTranslation());
        nonErrorTags++;
      }
    }
    double avgDistance = totalDistance / nonErrorTags;

    avgDistance *= tagDistanceScalar;

    double xyStdDev = Math.pow(avgDistance, tagDistancePower) / nonErrorTags;
    double thetaStdDev = Math.pow(avgDistance, tagDistancePower) / nonErrorTags;

    if (avgDistance >= trustCutOff) {
      return VecBuilder.fill(10000, 10000, 10000);
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public Optional<TimestampedPoseEstimator.TimestampedVisionUpdate> getLatestEstimate() {
    var raw = inputs.frame;

    if (preprocess != null) {
      raw = preprocess.apply(raw);
    }

    var estimate = photonPoseEstimator.update(raw, inputs.cameraMatrix, inputs.distanceCoeffs);

    return estimate.map(postprocess);
  }

  private PVApriltagIO getNewIO(ShamLibConstants.BuildMode buildMode, String name) {
    return switch (buildMode) {
      case REPLAY -> new PVApriltagIO() {};
      default -> new PVApriltagIOReal(name);
    };
  }
}
