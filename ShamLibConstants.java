package frc.robot.ShamLib;

public class ShamLibConstants {

  public static class SMF {
    public static final double TRANSITIONTIMEOUT = 2; // seconds

    // Change this value to change whether the state choosers will get sent
    public static boolean SEND_STATE_CHOOSERS = true;
  }

  public static class Swerve {
    // How far the module can be off from the actual position to trigger a correction
    public static final double ALLOWED_MODULE_ERROR = 2; // Deg
    // How close the modules must stay to reset the modules successfully
    public static final double ALLOWED_STOPPED_MODULE_DIFF = .2; // degrees
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
