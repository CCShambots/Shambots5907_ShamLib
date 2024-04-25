package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

public class DriveCommand extends Command {
  private final SwerveDrive drivetrain;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier turnSupplier;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  private final Double maxLinearSpeed;
  private final Double maxRotationalSpeed;

  private final double deadband;
  private final UnaryOperator<Double> controllerConversion;

  public DriveCommand(
      SwerveDrive drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier turnSupplier,
      double deadband,
      UnaryOperator<Double> controllerConversion,
      Subsystem subsystem,
      SwerveSpeedLimits speedLimits) {
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;

    xLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    yLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    thetaLimiter = new SlewRateLimiter(speedLimits.getMaxRotationalAcceleration());

    maxLinearSpeed = speedLimits.getMaxSpeed();
    maxRotationalSpeed = speedLimits.getMaxRotationalSpeed();

    this.deadband = deadband;
    this.controllerConversion = controllerConversion;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    resetSpeedLimiters();
  }

  @Override
  public void execute() {

    double correctedX = convertRawInput(xSupplier.getAsDouble()) * maxLinearSpeed;
    double correctedY = convertRawInput(ySupplier.getAsDouble()) * maxLinearSpeed;
    double correctedRot = convertRawInput(turnSupplier.getAsDouble()) * maxRotationalSpeed;

    ChassisSpeeds speeds;

    if (drivetrain.isFieldRelative()) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              correctedX, correctedY, correctedRot, drivetrain.getCurrentFieldOrientedAngle());
    } else {
      speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);
    }

    speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);
    speeds.omegaRadiansPerSecond = thetaLimiter.calculate(speeds.omegaRadiansPerSecond);

    drivetrain.drive(speeds, maxLinearSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double convertRawInput(double rawInput) {
    double deadbandInput = deadband(rawInput, deadband);
    return controllerConversion.apply(Double.valueOf(deadbandInput));
  }

  private double deadband(double rawInput, double deadband) {
    if (Math.abs(rawInput) > deadband) {
      if (rawInput > 0.0) return (rawInput - deadband) / (1.0 - deadband);
      else return (rawInput + deadband) / (1.0 - deadband);
    } else return 0;
  }

  private void resetSpeedLimiters() {
    ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();
    xLimiter.reset(currentSpeeds.vxMetersPerSecond);
    yLimiter.reset(currentSpeeds.vyMetersPerSecond);
    thetaLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
  }
}
