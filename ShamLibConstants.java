package frc.robot.ShamLib;

public class ShamLibConstants {

  public static class SMF {
    public static final double transitionTimeout = 2; // seconds
  }

  public static class Swerve {
    // How far the module can be off from the actual position to trigger a correction
    public static final double ALLOWED_MODULE_ERROR = 2; // Deg
    // How close the modules must stay to reset the modules successfully
    public static final double ALLOWED_STOPPED_MODULE_DIFF = .2; // degrees

    // Minimum target for a stall to be potentially detected
    public static final double MINIMUM_STALL_TARGET_VELOCITY = 1; // m/s

    // Minimum delta velocity for a stall to be triggered
    public static final double MINIMUM_STALL_DELTA_VELOCITY = 1; // m/s
  }

  public static enum BuildMode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
