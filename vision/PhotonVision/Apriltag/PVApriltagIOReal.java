package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PVApriltagIOReal implements PVApriltagIO {
  private final PhotonCamera camera;

  public PVApriltagIOReal(String camName) {
    camera = new PhotonCamera(camName);

    camera.setDriverMode(false);
  }

  @Override
  public void updateInputs(PVApriltagInputs inputs) {
    PhotonPipelineResult res = camera.getLatestResult();
    inputs.frame = res;

    inputs.cameraMatrix = camera.getCameraMatrix();
    inputs.distanceCoeffs = camera.getDistCoeffs();

    inputs.isConnected = camera.isConnected();

    inputs.timeStampSeconds = res.getTimestampSeconds();
  }
}
