package frc.robot.ShamLib.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CalculateWheelRadiusCommand extends Command {
  SlewRateLimiter thetaRateLimiter = new SlewRateLimiter(1);
  SwerveDrive drive;

  private Rotation2d startYaw = new Rotation2d();
  private SwerveModulePosition[] startStates = new SwerveModulePosition[4];
  private Timer timer = new Timer();
  private double assumedWheelRadius;
  private double driveBaseRadius;

  public CalculateWheelRadiusCommand(
      SwerveDrive drive, double assumedWheelRadius, double driveBaseRadius) {
    this.drive = drive;
    this.assumedWheelRadius = assumedWheelRadius;
    this.driveBaseRadius = driveBaseRadius;
  }

  @Override
  public void initialize() {
    startYaw = drive.getGyroHeading();
    thetaRateLimiter.reset(0);
    timer.restart();
    startStates = drive.getModulePositions();
  }

  @Override
  public void execute() {
    if (!timer.hasElapsed(2)) {
      drive.drive(new ChassisSpeeds(0, 0, thetaRateLimiter.calculate(1)));
    } else {
      drive.drive(new ChassisSpeeds(0, 0, thetaRateLimiter.calculate(0)));
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2) && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.001;
  }

  @Override
  public void end(boolean interrupted) {
    SwerveModulePosition[] endStates = drive.getModulePositions();
    Rotation2d endYaw = drive.getGyroHeading();

    double totalDelta = 0;

    for (int i = 0; i < endStates.length; i++) {
      double delta = endStates[i].distanceMeters - startStates[i].distanceMeters;
      totalDelta += Math.abs(delta);
    }

    double calculatedDistanceMeters = totalDelta / endStates.length;

    double actualDistanceMeters =
        Math.abs(startYaw.getRadians() - endYaw.getRadians()) * driveBaseRadius;

    double ratio = calculatedDistanceMeters / actualDistanceMeters;

    drive.stopModules();

    System.out.println("Calculated wheel radius in meters: " + ratio * assumedWheelRadius);
  }
}
