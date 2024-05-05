package frc.robot.ShamLib.vision.PhotonVision.Apriltag;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PVApriltagIO {
  public class PVApriltagInputs implements LoggableInputs {
    public boolean isConnected = false;
    public PhotonPipelineResult frame;
    public Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
    public Optional<Matrix<N5, N1>> distanceCoeffs = Optional.empty();
    public double timeStampSeconds;

    @Override
    public void toLog(LogTable table) {
      Packet packet = new Packet(frame.getPacketSize());
      PhotonPipelineResult.serde.pack(packet, frame);
      table.put("rawPipelineResult", packet.getData());
      table.put("isConnected", isConnected);

      table.put("timestampSeconds", frame.getTimestampSeconds());

      cameraMatrix.ifPresent(n3N3Matrix -> table.put("cameraMatrix", n3N3Matrix.getData()));
      distanceCoeffs.ifPresent(n5N1Matrix -> table.put("distanceCoeffs", n5N1Matrix.getData()));
    }

    @Override
    public void fromLog(LogTable table) {
      Packet packet = new Packet(table.get("rawPipelineResult", new byte[0]));
      frame = PhotonPipelineResult.serde.unpack(packet);

      timeStampSeconds = table.get("timestampSeconds", -1.00);
      frame.setTimestampSeconds(timeStampSeconds);

      isConnected = table.get("isConnected", false);

      double[] cameraMatrix = table.get("cameraMatrix", new double[0]);
      double[] distanceCoeffs = table.get("distanceCoeffs", new double[0]);

      if (cameraMatrix.length > 0) {
        this.cameraMatrix = Optional.of(MatBuilder.fill(Nat.N3(), Nat.N3(), cameraMatrix));
      } else {
        this.cameraMatrix = Optional.empty();
      }

      if (distanceCoeffs.length > 0) {
        this.distanceCoeffs = Optional.of(MatBuilder.fill(Nat.N5(), Nat.N1(), distanceCoeffs));
      } else {
        this.distanceCoeffs = Optional.empty();
      }
    }
  }

  public default void updateInputs(PVApriltagInputs inputs) {}
}
