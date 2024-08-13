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

  private final PIDGains translationGains;
  private final PIDGains autoThetaGains;
  private final double driveBaseRadius; // In meters

  private final double loopPeriod;

  private final BooleanSupplier flipTrajectory;
  private final Subsystem subsystem;

  protected OdometryBoundingBox odometryBoundingBox;

  /**
   * Constructor for your typical swerve drive with odometry compatible with vision pose estimation
   *
   * @param mode Build mode for AdvantageKit related logging
   * @param pigeon2ID CAN id of the pigeon 2 gyro
   * @param moduleDriveGains PIDSV gains for the velocity of the swerve modules
   * @param moduleTurnGains PIDSV gains for the position of the swerve modules
   * @param speedLimits Default chassis speed limits
   * @param maxModuleTurnVelo Maximum velocity of the turn motors
   * @param maxModuleTurnAccel Maximum acceleration of the turn motors
   * @param autoThetaGains PID gains for the autonomous angle controller (configured with
   *     PathPlanner)
   * @param translationGains PID gains for the translation of the bot (used with PathPlanner)
   * @param moduleCanbus The canbus the swerve modules are on (pass "" for RIO and "*" for a
   *     CANivore)
   * @param gyroCanbus The gryo the swerve modules are on (pass "" for RIO and "*" for a CANivore)
   * @param currentLimit The current limits to apply to the motors
   * @param subsystem The drivetrain subsystem this object is constructed in
   * @param flipTrajectory Whether to flip the trajectory (blue vs. red alliance)
   * @param loopPeriod RIO loop time (used to discretize chassis speeds, preventing "coriolis"
   *     effect of turning)
   * @param moduleInfos Data about each swerve module
   */
  @Deprecated
  public SwerveDrive(
      BuildMode mode,
      int pigeon2ID,
      PIDSVGains moduleDriveGains,
      PIDSVGains moduleTurnGains,
      SwerveSpeedLimits speedLimits,
      double maxModuleTurnVelo,
      double maxModuleTurnAccel,
      PIDGains autoThetaGains,
      PIDGains translationGains,
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
        speedLimits,
        maxModuleTurnVelo,
        maxModuleTurnAccel,
        autoThetaGains,
        translationGains,
        moduleCanbus,
        gyroCanbus,
        currentLimit,
        subsystem,
        flipTrajectory,
        null,
        loopPeriod,
        moduleInfos);
  }

  /**
   * More detailed constructor
   *
   * @param mode Build mode for AdvantageKit related logging
   * @param pigeon2ID CAN id of the pigeon 2 gyro
   * @param moduleDriveGains PIDSV gains for the velocity of the swerve modules
   * @param moduleTurnGains PIDSV gains for the position of the swerve modules
   * @param speedLimits Default chassis speed limits
   * @param maxModuleTurnVelo Maximum velocity of the turn motors
   * @param maxModuleTurnAccel Maximum acceleration of the turn motors
   * @param autoThetaGains PID gains for the autonomous angle controller (configured with
   *     PathPlanner)
   * @param translationGains PID gains for the translation of the bot (used with PathPlanner)
   * @param moduleCanbus The canbus the swerve modules are on (pass "" for RIO and "*" for a
   *     CANivore)
   * @param gyroCanbus The gryo the swerve modules are on (pass "" for RIO and "*" for a CANivore)
   * @param currentLimit The current limits to apply to the motors
   * @param subsystem The drivetrain subsystem this object is constructed in
   * @param useTimestamped Whether to use timestamped vision data
   * @param flipTrajectory Whether to flip the trajectory (blue vs. red alliance)
   * @param stdDevs Standard deviations for the timestamped pose estimator
   * @param loopPeriod RIO loop time (used to discretize chassis speeds, preventing "coriolis"
   *     effect of turning)
   * @param moduleInfos Data about each swerve module
   */
  @Deprecated
  public SwerveDrive(
      BuildMode mode,
      int pigeon2ID,
      PIDSVGains moduleDriveGains,
      PIDSVGains moduleTurnGains,
      SwerveSpeedLimits speedLimits,
      double maxModuleTurnVelo,
      double maxModuleTurnAccel,
      PIDGains autoThetaGains,
      PIDGains translationGains,
      String moduleCanbus,
      String gyroCanbus,
      CurrentLimitsConfigs currentLimit,
      Subsystem subsystem,
      BooleanSupplier flipTrajectory,
      Matrix<N3, N1> stdDevs,
      double loopPeriod,
      ModuleInfo... moduleInfos) {

    this.buildMode = mode;

    this.maxChassisSpeed = speedLimits.getMaxSpeed();
    this.maxChassisAcceleration = speedLimits.getMaxAcceleration();
    this.maxChassisRotationVel = speedLimits.getMaxRotationalSpeed();
    this.maxChassisRotationAccel = speedLimits.getMaxRotationalAcceleration();

    this.translationGains = translationGains;

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
    }

    kDriveKinematics = new SwerveDriveKinematics(offsets);

    // Logic for changing what IO objects are constructed based on the build mode
    switch (mode) {
      case REAL:
        gyroIO = new GyroIOReal(pigeon2ID, gyroCanbus);
        odometry =
            new SwerveOdometryReal(
                new SwerveDrivePoseEstimator(
                    kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));

        break;
      case REPLAY:
        gyroIO = new GyroIO() {};
        odometry =
            new SwerveOdometryReal(
                new SwerveDrivePoseEstimator(
                    kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));
        break;
      default:
        odometry = new SwerveOdometrySim(kDriveKinematics, modules);

        gyroIO = new GyroIO() {};
        break;
    }

    rotationOffset = getGyroHeading();
    holdAngle = new Rotation2d(rotationOffset.getRadians());

    this.driveBaseRadius =
        Math.hypot(
            moduleInfos[0].offset.getX(),
            moduleInfos[0].offset.getY()); // Radius of the drive base in meters

    this.flipTrajectory = flipTrajectory;
    this.subsystem = subsystem;
    this.autoThetaGains = autoThetaGains;
  }

  public SwerveDrive(SwerveDriveConfig config) {

    this.buildMode = config.buildMode;

    this.maxChassisSpeed = config.standardSpeedLimits.getMaxSpeed();
    this.maxChassisAcceleration = config.standardSpeedLimits.getMaxAcceleration();
    this.maxChassisRotationVel = config.standardSpeedLimits.getMaxRotationalSpeed();
    this.maxChassisRotationAccel = config.standardSpeedLimits.getMaxRotationalAcceleration();

    this.translationGains = config.translationGains;

    this.loopPeriod = config.loopPeriod;

    modules = new ArrayList<>();
    Translation2d[] offsets = new Translation2d[config.moduleInfos.length];
    for (int i = 0; i < config.moduleInfos.length; i++) {
      numModules++;
      ModuleInfo m = config.moduleInfos[i];
      offsets[i] = m.offset;

      SwerveModuleIO io = new SwerveModuleIO() {};

      switch (config.buildMode) {
        case REAL:
          io =
              new SwerveModuleIOReal(
                  config.moduleCanbus,
                  m,
                  config.moduleDriveGains,
                  config.moduleTurnGains,
                  config.maxModuleTurnVelo,
                  config.maxModuleTurnAccel,
                  m.enableTurnFOC,
                  m.enableTurnFOC,
                  config.currentLimit);
          break;
        case SIM:
          io =
              new SwerveModuleIOSim(
                  config.moduleCanbus,
                  m,
                  config.moduleDriveGains,
                  config.moduleTurnGains,
                  config.maxModuleTurnVelo,
                  config.maxModuleTurnAccel,
                  config.currentLimit);
        default:
          break;
      }

      modules.add(
          new SwerveModule(
              io,
              "Module-" + numModules,
              config.moduleCanbus,
              m,
              config.moduleDriveGains,
              config.moduleTurnGains,
              config.maxModuleTurnVelo,
              config.maxModuleTurnAccel));
    }

    kDriveKinematics = new SwerveDriveKinematics(offsets);

    // Logic for changing what IO objects are constructed based on the build mode
    switch (config.buildMode) {
      case REAL:
        gyroIO = new GyroIOReal(config.pigeon2ID, config.gyroCanbus);
        odometry =
            new SwerveOdometryReal(
                new SwerveDrivePoseEstimator(
                    kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));

        break;
      case REPLAY:
        gyroIO = new GyroIO() {};
        odometry =
            new SwerveOdometryReal(
                new SwerveDrivePoseEstimator(
                    kDriveKinematics, getCurrentAngle(), getModulePositions(), new Pose2d()));
        break;
      default:
        odometry = new SwerveOdometrySim(kDriveKinematics, modules);

        gyroIO = new GyroIO() {};
        break;
    }

    rotationOffset = getGyroHeading();
    holdAngle = new Rotation2d(rotationOffset.getRadians());

    this.driveBaseRadius =
        Math.hypot(
            config.moduleInfos[0].offset.getX(),
            config.moduleInfos[0].offset.getY()); // Radius of the drive base in meters

    this.flipTrajectory = config.flipTrajectory;
    this.subsystem = config.subsystem;
    this.autoThetaGains = config.autoThetaGains;
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

    // Discretizing chassis speeds helps to avoid "pulling"/"coriolis effect"
    // that happens when the drivetrain tries to turn at high speeds
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

  /** Align all modules straight forwards */
  public void alignModules() {
    setAllModules(new SwerveModuleState(0, new Rotation2d()));
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

  /* RESET COMMANDS FOR DIFFERENT ASPECTS */

  /**
   * WARNING: This does NOT work with odometry It causes the bot's rotation to arbitrarily change
   */
  public void resetGyro(Rotation2d angle) {
    gyroIO.setGyroYaw(angle);
    fieldOrientedRotationOffset = new Rotation2d();
    holdAngle = angle;
  }

  public void resetGyro() {
    resetGyro(new Rotation2d());
  }

  /**
   * This is what should be used with odometry. It applies an offset that is only used for field
   * oriented driving, rather than screwing with a gyro value
   */
  public void resetFieldOrientedRotationOffset(Rotation2d angle) {
    fieldOrientedRotationOffset = angle;
  }

  public void resetOdometryPose(Pose2d newPose) {
    odometry.resetPose(getCurrentAngle(), newPose, getModulePositions());
  }

  public void resetOdometryPose() {
    resetOdometryPose(new Pose2d());
  }

  public List<SwerveModule> getModules() {
    return modules;
  }

  @AutoLogOutput
  public boolean isStalling() {
    return modules.stream().anyMatch(SwerveModule::isStalling);
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
