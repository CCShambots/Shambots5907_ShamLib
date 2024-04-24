package frc.robot.ShamLib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.swerve.gyro.GyroIO;
import frc.robot.ShamLib.swerve.gyro.GyroIOReal;
import frc.robot.ShamLib.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import frc.robot.ShamLib.swerve.module.SwerveModuleIO;
import frc.robot.ShamLib.swerve.module.SwerveModuleIOReal;
import frc.robot.ShamLib.swerve.module.SwerveModuleIOSim;
import frc.robot.ShamLib.swerve.odometry.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive {

  protected final List<SwerveModule> modules;

  protected final GyroIO gyroIO;
  protected final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();

  private final BuildMode buildMode;

  protected final SwerveOdometry odometry;

  protected final SwerveDriveKinematics kDriveKinematics;
  protected final double maxChassisSpeed;
  protected final double maxChassisAcceleration;
  protected final double maxChassisRotationVel;
  protected final double maxChassisRotationAccel;
  private int numModules = 0;
  @AutoLogOutput private Rotation2d rotationOffset = new Rotation2d();
  @AutoLogOutput private Rotation2d fieldOrientedRotationOffset = new Rotation2d();
  private Rotation2d holdAngle;

  private boolean fieldRelative = true;

  private final Field2d field;
  private final boolean extraTelemetry;

  private int speedMode = 0;

  private final PIDGains translationGains;
  private final PIDGains rotationGains;
  private final double driveBaseRadius; // In meters

  private final double loopPeriod;

  private final BooleanSupplier flipTrajectory;
  private final PIDGains autoThetaGains;
  private final Subsystem subsystem;

  protected OdometryBoundingBox odometryBoundingBox;

  /**
   * Constructor for your typical swerve drive with odometry compatible with vision pose estimation
   *
   * @param pigeon2ID CAN idea of the pigeon 2 gyro
   * @param moduleDriveGains PIDSV gains for the velocity of the swerve modules
   * @param moduleTurnGains PIDSV gains for the position of the swerve modules
   * @param maxModuleTurnVelo maximum velocity the turn motors should go
   * @param maxModuleTurnAccel maximum acceleration the turn motors should go
   * @param autoThetaGains PID gains for the angle hold controller in autonomous
   * @param translationGains PID gains for the trans
   * @param extraTelemetry whether to send additional telemetry data, like vision pose measurements,
   *     trajectory data, and module poses
   * @param moduleCanbus The canbus the modules are on (pass "" for default)
   * @param gyroCanbus The canbus the gyro is on (pass "" for default)
   * @param moduleInfos Array of module infos, one for each module
   */
  public SwerveDrive(
      BuildMode mode,
      int pigeon2ID,
      PIDSVGains moduleDriveGains,
      PIDSVGains moduleTurnGains,
      double maxChassisSpeed,
      double maxChassisAccel,
      double maxChassisRotationVel,
      double maxChassisRotationAccel,
      double maxModuleTurnVelo,
      double maxModuleTurnAccel,
      PIDGains autoThetaGains,
      PIDGains translationGains,
      boolean extraTelemetry,
      String moduleCanbus,
      String gyroCanbus,
      CurrentLimitsConfigs currentLimit,
      Subsystem subsystem,
      BooleanSupplier flipTrajectory,
      double loopPeriod,
      ModuleInfo... moduleInfos) {
    this(
        mode,
        pigeon2ID,
        moduleDriveGains,
        moduleTurnGains,
        maxChassisSpeed,
        maxChassisAccel,
        maxChassisRotationVel,
        maxChassisRotationAccel,
        maxModuleTurnVelo,
        maxModuleTurnAccel,
        autoThetaGains,
        translationGains,
        extraTelemetry,
        moduleCanbus,
        gyroCanbus,
        currentLimit,
        subsystem,
        false,
        flipTrajectory,
        null,
        loopPeriod,
        moduleInfos);
  }

  /**
   * Constructor for your typical swerve drive with odometry compatible with vision pose estimation
   *
   * @param pigeon2ID CAN idea of the pigeon 2 gyro
   * @param moduleDriveGains PIDSV gains for the velocity of the swerve modules
   * @param moduleTurnGains PIDSV gains for the position of the swerve modules
   * @param maxModuleTurnVelo maximum velocity the turn motors should go
   * @param maxModuleTurnAccel maximum acceleration the turn motors should go
   * @param autoThetaGains PID gains for the angle hold controller in autonomous
   * @param translationGains PID gains for the trans
   * @param extraTelemetry whether to send additional telemetry data, like vision pose measurements,
   *     trajectory data, and module poses
   * @param moduleCanbus The canbus the modules are on (pass "" for default)
   * @param gyroCanbus The canbus the gyro is on (pass "" for default)
   * @param moduleInfos Array of module infos, one for each module
   */
  public SwerveDrive(
      BuildMode mode,
      int pigeon2ID,
      PIDSVGains moduleDriveGains,
      PIDSVGains moduleTurnGains,
      double maxChassisSpeed,
      double maxChassisAccel,
      double maxChassisRotationVel,
      double maxChassisRotationAccel,
      double maxModuleTurnVelo,
      double maxModuleTurnAccel,
      PIDGains autoThetaGains,
      PIDGains translationGains,
      boolean extraTelemetry,
      String moduleCanbus,
      String gyroCanbus,
      CurrentLimitsConfigs currentLimit,
      Subsystem subsystem,
      boolean useTimestamped,
      BooleanSupplier flipTrajectory,
      Matrix<N3, N1> stdDevs,
      double loopPeriod,
      ModuleInfo... moduleInfos) {

    this.buildMode = mode;

    this.extraTelemetry = extraTelemetry;
    this.maxChassisSpeed = maxChassisSpeed;
    this.maxChassisAcceleration = maxChassisAccel;
    this.maxChassisRotationVel = maxChassisRotationVel;
    this.maxChassisRotationAccel = maxChassisRotationAccel;

    this.translationGains = translationGains;
    this.rotationGains = autoThetaGains;

    this.loopPeriod = loopPeriod;

    modules = new ArrayList<>();
    Translation2d[] offsets = new Translation2d[moduleInfos.length];
    for (int i = 0; i < moduleInfos.length; i++) {
      numModules++;
      ModuleInfo m = moduleInfos[i];
      offsets[i] = m.offset;

      SwerveModuleIO io = new SwerveModuleIO() {};

      switch (mode) {
        case REAL:
          io =
              new SwerveModuleIOReal(
                  moduleCanbus,
                  m,
                  moduleDriveGains,
                  moduleTurnGains,
                  maxModuleTurnVelo,
                  maxModuleTurnAccel,
                  m.enableTurnFOC,
                  m.enableTurnFOC,
                  currentLimit);
          break;
        case SIM:
          io =
              new SwerveModuleIOSim(
                  moduleCanbus,
                  m,
                  moduleDriveGains,
                  moduleTurnGains,
                  maxModuleTurnVelo,
                  maxModuleTurnAccel,
                  currentLimit);
        default:
          break;
      }

      modules.add(
          new SwerveModule(
              io,
              "Module-" + numModules,
              moduleCanbus,
              m,
              moduleDriveGains,
              moduleTurnGains,
              maxModuleTurnVelo,
              maxModuleTurnAccel));

      if (extraTelemetry) {
        SmartDashboard.putData("Module-" + i, modules.get(i));
      }
    }

    kDriveKinematics = new SwerveDriveKinematics(offsets);

    switch (mode) {
      case REAL:
        gyroIO = new GyroIOReal(pigeon2ID, gyroCanbus);
        if (!useTimestamped) {
          odometry =
              new SwerveOdometryReal(
                  new SwerveDrivePoseEstimator(
                      kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));
        } else {
          odometry =
              new SwerveTimestampedOdometryReal(
                  new TimestampedPoseEstimator(stdDevs),
                  kDriveKinematics,
                  modules,
                  this::getCurrentAngle);
        }
        break;
      case REPLAY:
        gyroIO = new GyroIO() {};
        odometry =
            new SwerveOdometryReal(
                new SwerveDrivePoseEstimator(
                    kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));
        break;
      default:
        if (!useTimestamped) {
          odometry = new SwerveOdometrySim(kDriveKinematics, modules);
        } else {
          odometry =
              new SwerveTimestampedOdometrySim(
                  new TimestampedPoseEstimator(stdDevs), kDriveKinematics, modules);
        }
        gyroIO = new GyroIO() {};
        break;
    }

    rotationOffset = getGyroHeading();
    holdAngle = new Rotation2d(rotationOffset.getRadians());

    field = new Field2d();

    this.driveBaseRadius =
        Math.hypot(
            moduleInfos[0].offset.getX(),
            moduleInfos[0].offset.getY()); // Radius of the drive base in meters

    this.flipTrajectory = flipTrajectory;
    this.subsystem = subsystem;
    this.autoThetaGains = autoThetaGains;
  }

  public void configurePathplanner() {
    // Configure the auto builder stuff
    if (!AutoBuilder.isConfigured()) {
      AutoBuilder.configureHolonomic(
          this::getPose,
          this::resetOdometryPose,
          this::getChassisSpeeds,
          this::drive,
          new HolonomicPathFollowerConfig(
              translationGains.toPIDConstants(),
              autoThetaGains.toPIDConstants(),
              maxChassisSpeed,
              driveBaseRadius,
              new ReplanningConfig()),
          flipTrajectory,
          subsystem);
    }

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  /*MUST BE CALLED PERIODICALLY */
  public void update() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    for (SwerveModule m : modules) {
      m.update();
    }

    updateOdometry();
  }

  public Rotation2d getPitch() {
    return gyroInputs.gyroPitch;
  }

  public Rotation2d getRoll() {
    return gyroInputs.gyroRoll;
  }

  public void addTimestampedVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> measurements) {
    odometry.addTimestampedVisionMeasurements(measurements);
  }

  public void addVisionMeasurement(Pose2d pose) {
    if (extraTelemetry) field.getObject("vision").setPose(pose);

    odometry.addVisionMeasurement(pose);
  }

  /** Updates the odometry pose estimator. THIS MUST BE CALLED PERIODICALLY */
  private void updateOdometry() {
    switch (buildMode) {
      case SIM:
        // Update the odometry for sim (everything it needs is already passed in in the constructor)
        odometry.updatePose();
        break;
      default:
        // Update odometry normally if being fed data from a real robot
        odometry.updatePose(getCurrentAngle(), getModulePositions());
        break;
    }

    if (odometryBoundingBox != null) {
      Pose2d corrected = odometryBoundingBox.correctPose(getPose());

      if (!corrected.equals(getPose())) {
        resetOdometryPose(corrected);
      }
    }
  }

  public void setOdometryBoundingBox(OdometryBoundingBox odometryBoundingBox) {
    this.odometryBoundingBox = odometryBoundingBox;
  }

  public Command getCalculateWheelRadiusCommand(double assumedWheelRadiusMeters) {
    return new CalculateWheelRadiusCommand(this, assumedWheelRadiusMeters, driveBaseRadius);
  }

  public double[] getModuleAngles() {

    double[] angles = new double[numModules];

    for (int i = 0; i < modules.size(); i++) {
      angles[i] = modules.get(i).getCurrentState().angle.getDegrees();
    }

    return angles;
  }

  @AutoLogOutput(key = "SwerveDrive/Module States")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[numModules];

    for (int i = 0; i < modules.size(); i++) {
      states[i] = modules.get(i).getCurrentState();
    }

    return states;
  }

  @AutoLogOutput(key = "SwerveDrive/Module Targets")
  public SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[numModules];

    for (int i = 0; i < modules.size(); i++) {
      states[i] = modules.get(i).getTargetState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[numModules];

    for (int i = 0; i < modules.size(); i++) {
      positions[i] = modules.get(i).getCurrentPosition();
    }

    return positions;
  }

  public double[] getModuleAbsoluteAngles() {
    double[] out = new double[4];

    for (int i = 0; i < modules.size(); i++) {
      out[i] = modules.get(i).getAbsoluteAngle().getDegrees();
    }

    return out;
  }

  /**
   * Method to call to update the states of the swerve drivetrain
   *
   * @param speeds chassis speed object to move
   */
  public void drive(ChassisSpeeds speeds, double maxChassisSpeed) {
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    if (linearSpeed > maxChassisSpeed) {
      double factor = maxChassisSpeed / linearSpeed;

      speeds.vxMetersPerSecond = speeds.vxMetersPerSecond * factor;
      speeds.vyMetersPerSecond = speeds.vyMetersPerSecond * factor;
    }

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, loopPeriod);

    SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxChassisSpeed);

    setModuleStates(swerveModuleStates);
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, maxChassisSpeed);
  }

  /**
   * Sets the target state of each swerve module based on the input array
   *
   * @param states array of swerve module states
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules.get(i).setDesiredState(states[i]);
    }
  }

  public void setAllModules(SwerveModuleState state) {
    for (SwerveModule module : modules) {
      module.setDesiredState(state);
    }
  }

  /** Align all modules straight forwards */
  public void alignModules() {
    setAllModules(new SwerveModuleState(0, new Rotation2d()));
  }

  /**
   * Finds the angle of the robot in radians (limited -PI to PI)
   *
   * @return Robot angle
   */
  @AutoLogOutput(key = "SwerveDrive/Current Angle")
  public Rotation2d getCurrentAngle() {
    if (buildMode == BuildMode.SIM) {
      return getPose().getRotation();
    }
    return getGyroHeading().minus(rotationOffset);
  }

  @AutoLogOutput(key = "SwerveDrive/Field Oriented Angle")
  public Rotation2d getCurrentFieldOrientedAngle() {
    return getCurrentAngle().minus(fieldOrientedRotationOffset);
  }

  public void stopModules() {
    modules.forEach(SwerveModule::stop);
  }

  /**
   * Get a command to run a path-planner trajectory on the swerve drive
   *
   * @param path the trajectory to run
   * @return the command to run
   */
  public Command getPathCommand(PathPlannerPath path) {

    return AutoBuilder.followPath(path);
  }

  public Command getTrajectoryCommand(PathPlannerPath trajectory) {
    return getPathCommand(trajectory);
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public Rotation2d getGyroHeading() {
    return gyroInputs.gyroYaw;
  }

  public Rotation2d getHoldAngle() {
    return holdAngle;
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public void setFieldRelative(boolean value) {
    fieldRelative = value;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getTargetChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getTargetModuleStates());
  }

  public int getSpeedMode() {
    return speedMode;
  }

  public void setSpeedMode(int speedMode) {
    this.speedMode = speedMode;
  }

  /* RESET COMMANDS FOR DIFFERENT ASPECTS */

  /**
   * WARNING: This does NOT work with odometry It causes the bot's rotation to arbitrarily change
   */
  public void resetGyro(Rotation2d angle) {
    gyroIO.setGyroYaw(angle);
    fieldOrientedRotationOffset = new Rotation2d();
    holdAngle = angle;
  }

  public void resetFieldOrientedRotationOffset(Rotation2d angle) {
    fieldOrientedRotationOffset = angle;
  }

  public void resetGyro() {
    resetGyro(new Rotation2d());
  }

  public void resetOdometryPose(Pose2d newPose) {
    odometry.resetPose(getCurrentAngle(), newPose, getModulePositions());
  }

  public void resetOdometryPose() {
    resetOdometryPose(new Pose2d());
  }

  public Field2d getField() {
    return field;
  }

  public List<SwerveModule> getModules() {
    return modules;
  }

  public Command createPathFindingCommand(Pose2d target) {
    PathConstraints constraints =
        new PathConstraints(
            maxChassisSpeed,
            maxChassisAcceleration,
            maxChassisRotationVel,
            maxChassisRotationAccel);

    return AutoBuilder.pathfindToPose(target, constraints, 0);
  }

  public Command getTurnVoltageCalcCommand(
      Trigger stop, Trigger incrementUp, Trigger incrementDown, double incrementAmount) {
    return modules
        .get(0)
        .getTurnVoltageCalcCommand(stop, incrementUp, incrementDown, incrementAmount);
  }

  public Command getDriveVoltageCalcCommand(
      Trigger stop, Trigger incrementUp, Trigger incrementDown, double incrementAmount) {
    Command[] tuningCommands =
        modules.stream()
            .map(
                (m) ->
                    m.getDriveVoltageCalcCommand(stop, incrementUp, incrementDown, incrementAmount))
            .toArray(Command[]::new);

    return new SequentialCommandGroup(
        // set all modules to the same angle and 0 speed
        new InstantCommand(() -> setAllModules(new SwerveModuleState())),
        // run drive tuning on all modules in parallel
        new ParallelCommandGroup(tuningCommands));
  }
}
