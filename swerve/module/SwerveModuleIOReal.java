package frc.robot.ShamLib.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class SwerveModuleIOReal implements SwerveModuleIO {

  protected final MotionMagicTalonFX turnMotor;
  protected final VelocityTalonFX driveMotor;

  private final CANcoder turnEncoder;

  public SwerveModuleIOReal(
      String canbus,
      ModuleInfo mI,
      PIDSVGains driveGains,
      PIDSVGains turnGains,
      double maxTurnVelo,
      double maxTurnAccel,
      boolean enableDriveFOC,
      boolean enableTurnFOC,
      CurrentLimitsConfigs currentLimit) {

    this.turnEncoder = new CANcoder(mI.encoderID, canbus);
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    this.turnEncoder.getConfigurator().apply(canCoderConfig);

    turnMotor =
        new MotionMagicTalonFX(
            mI.turnMotorID, canbus, turnGains, mI.turnRatio, maxTurnVelo, maxTurnAccel);
    turnMotor.setInverted(mI.turnInverted); // All turn modules were inverted
    applyCurrentLimit(turnMotor, currentLimit);

    driveMotor = new VelocityTalonFX(mI.driveMotorID, canbus, driveGains, mI.driveRatio);
    driveMotor.setInverted(mI.driveInverted);
    applyCurrentLimit(driveMotor, currentLimit);

    MotorOutputConfigs configs = new MotorOutputConfigs();
    driveMotor.getConfigurator().refresh(configs);
    configs.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(configs);

    turnMotor.setFOC(enableTurnFOC);
    driveMotor.setFOC(enableDriveFOC);
  }

  @Override
  public void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.CurrentLimits = limit;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.turnEncoderPos = turnEncoder.getAbsolutePosition().getValue() * 360;

    inputs.turnMotorAngle = turnMotor.getEncoderPosition();
    inputs.turnMotorVelocity = turnMotor.getEncoderVelocity();
    inputs.turnMotorTarget = turnMotor.getTarget();
    inputs.turnMotorVoltage = turnMotor.getMotorVoltage().getValue();

    inputs.driveMotorPosition = driveMotor.getEncoderPosition();
    inputs.driveMotorVelocity = driveMotor.getEncoderVelocity();
    inputs.driveMotorVoltage = driveMotor.getMotorVoltage().getValue();
    inputs.driveMotorTarget = driveMotor.getTarget();

    inputs.driveMotorRotorVelocity = driveMotor.getRotorVelocity().getValueAsDouble();
    inputs.turnMotorRotorVelocity = turnMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void stop() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnMotorVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public void resetTurnMotorPosition(double pos) {
    turnMotor.resetPosition(pos);
  }

  @Override
  public void setDriveMotorTarget(double target) {
    driveMotor.setTarget(target);
  }

  @Override
  public void setTurnMotorTarget(double target) {
    turnMotor.setTarget(target);
  }
}
