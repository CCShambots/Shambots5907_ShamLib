package frc.robot.ShamLib.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ModuleInfo {

  public final int driveMotorID;
  public final int turnMotorID;
  public final int encoderID;
  public final Rotation2d encoderOffset;
  public final Translation2d offset;
  public final double turnRatio;
  public final double driveRatio;
  public final boolean driveInverted;
  public final boolean turnInverted;
  public final boolean enableDriveFOC;
  public final boolean enableTurnFOC;

  public ModuleInfo(
      int driveMotorID,
      int turnMotorID,
      int encoderID,
      Rotation2d encoderOffset,
      Translation2d offset,
      double turnRatio,
      double driveRatio,
      boolean driveInverted,
      boolean turnInverted,
      boolean enableDriveFOC,
      boolean enableTurnFOC) {
    this.driveMotorID = driveMotorID;
    this.turnMotorID = turnMotorID;
    this.encoderID = encoderID;
    this.encoderOffset = encoderOffset;
    this.offset = offset;
    this.turnRatio = turnRatio;
    this.driveRatio = driveRatio;
    this.driveInverted = driveInverted;
    this.turnInverted = turnInverted;
    this.enableDriveFOC = enableDriveFOC;
    this.enableTurnFOC = enableTurnFOC;
  }

  /**
   * @param type SDS Module type (i.e. MK4i)
   * @param speed Speed level of SDS module
   * @param driveMotorID CAN ID of the drive motor
   * @param turnMotorID CAN ID of the turn motor
   * @param encoderID CAN ID of the CANCoder
   * @param encoderOffset in degrees; should be set equal to the encoder reading
   * @param offset Translational offset of the swerve module from the robot's center
   * @param driveInverted whether the drive motor should be inverted from its standard direction
   * @return the module info to pass into the SwerveDrive
   */
  public static ModuleInfo generateModuleInfo(
      SwerveModuleType type,
      SwerveModuleSpeedLevel speed,
      double wheelRadius,
      int driveMotorID,
      int turnMotorID,
      int encoderID,
      double encoderOffset,
      Translation2d offset,
      boolean driveInverted,
      boolean enableDriveFOC,
      boolean enableTurnFOC) {
    return new ModuleInfo(
        driveMotorID,
        turnMotorID,
        encoderID,
        Rotation2d.fromDegrees(encoderOffset),
        offset,
        type.turnRatio,
        speed.gearRatio * 2 * Math.PI * wheelRadius,
        driveInverted,
        type.turnInverted,
        enableDriveFOC,
        enableTurnFOC);
  }

  public enum SwerveModuleType {
    MK4(false, (1.0 / 12.8) * 360),
    MK4i(true, (7.0 / 150.0) * 360);

    public final boolean turnInverted; // Whether the turn motor should be inverted
    public final double turnRatio; // The gear ratio on the turn motor (deg)

    private SwerveModuleType(boolean turnInverted, double turnRatio) {
      this.turnInverted = turnInverted;
      this.turnRatio = turnRatio;
    }
  }

  public enum SwerveModuleSpeedLevel {
    L1(1.0 / 8.14), // Slowest
    L2(1.0 / 6.75),
    L3(1.0 / 6.12),
    // NOTE: L4 doesn't exist for MK4i
    L4(1.0 / 5.14); // Fastest

    public final double gearRatio;

    private SwerveModuleSpeedLevel(double gearRatio) {
      this.gearRatio = gearRatio;
    }
  }
}
