// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shoulder;

import static frc.robot.subsystems.shoulder.Shoulder.FORWARD_SOFT_LIMIT_DEGREES;
import static frc.robot.subsystems.shoulder.Shoulder.REVERSE_SOFT_LIMIT_DEGREES;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.PortConfiguration;
import frc.robot.utils.drivers.Phoenix6Util;
import frc.robot.utils.drivers.TalonFXFactory;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

public class ShoulderIOCTRE implements ShoulderIO {
  private final TalonFX flLeaderMotor;
  private final TalonFX frFollowerMotor;
  private final TalonFX blFollowerMotor;
  private final TalonFX brFollowerMotor;
  private final CANcoder encoder;
  private static final double PIVOT_GEAR_RATIO = (135.0 / 1.0);
  private static final double MOTOR_ROTATIONS_PER_DEGREE = PIVOT_GEAR_RATIO / 360.0;
  private static final double DEGREES_PER_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_DEGREE;

  private final TalonFXConfiguration configuration = new TalonFXConfiguration();

  private final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0);

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer flLeaderDebounce = new Debouncer(0.5);
  private final Debouncer frFollowerDebounce = new Debouncer(0.5);
  private final Debouncer blFollowerDebounce = new Debouncer(0.5);
  private final Debouncer brFollowerDebounce = new Debouncer(0.5);
  private final Debouncer encoderDebounce = new Debouncer(0.5);

  // Lock for thread-safe access to sensor data
  private final ReentrantReadWriteLock sensorLock = new ReentrantReadWriteLock();

  // Cached sensor values for thread-safe access
  private double cachedShoulderPosition = 0.0;
  private double cachedShoulderVelocity = 0.0;
  private double cachedShoulderVoltage = 0.0;
  private double cachedShoulderCurrent = 0.0;
  private double cachedShoulderTemperature = 0.0;
  private double cachedEncoderPosition = 0.0;
  private double cachedEncoderVelocity = 0.0;

  public ShoulderIOCTRE(PortConfiguration ports) {
    flLeaderMotor = TalonFXFactory.createDefaultTalon(ports.flShoulderMotorID, false);
    frFollowerMotor =
        TalonFXFactory.createPermanentFollowerTalon(
            ports.frShoulderMotorID, ports.flShoulderMotorID, true);
    blFollowerMotor =
        TalonFXFactory.createPermanentFollowerTalon(
            ports.blShoulderMotorID, ports.flShoulderMotorID, false);
    brFollowerMotor =
        TalonFXFactory.createPermanentFollowerTalon(
            ports.brShoulderMotorID, ports.flShoulderMotorID, true);
    encoder = new CANcoder(ports.shoulderEncoderID.getDeviceNumber(), ports.CANBus);

    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    configuration.CurrentLimits.SupplyCurrentLimit = 60.0;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = 120;
    configuration.Audio.BeepOnConfig = false;

    configuration.Slot0.kP = 0.3;
    configuration.Slot0.kI = 0;
    configuration.Slot0.kD = 0;
    configuration.Slot0.kV = 0;
    configuration.Slot0.kA = 0;
    configuration.Slot0.kS = 0;

    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        FORWARD_SOFT_LIMIT_DEGREES * MOTOR_ROTATIONS_PER_DEGREE;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        REVERSE_SOFT_LIMIT_DEGREES * MOTOR_ROTATIONS_PER_DEGREE;

    // Start with soft limits disabled.  Enable them when homing.
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;

    Phoenix6Util.applyAndCheckConfiguration(flLeaderMotor, configuration);
  }

  @Override
  public void setSetpointInDegrees(double setpointInDegrees) {
    flLeaderMotor.setControl(request.withPosition(degreesToMotorRotations(setpointInDegrees)));
  }

  double degreesToMotorRotations(double degrees) {
    return degrees * MOTOR_ROTATIONS_PER_DEGREE;
  }

  @Override
  public void setHomingPosition(double position) {
    flLeaderMotor.setPosition(degreesToMotorRotations(position));
  }

  @Override
  public void disableSoftLimits() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    flLeaderMotor.getConfigurator().refresh(config);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    flLeaderMotor.getConfigurator().apply(config);
  }

  @Override
  public void enableSoftLimits() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    flLeaderMotor.getConfigurator().refresh(config);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    flLeaderMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    // Refresh all sensor data
    StatusCode flLeaderStatus =
        BaseStatusSignal.refreshAll(
            flLeaderMotor.getMotorVoltage(),
            flLeaderMotor.getSupplyCurrent(),
            flLeaderMotor.getDeviceTemp(),
            flLeaderMotor.getPosition(),
            flLeaderMotor.getVelocity());

    StatusCode frFollowerStatus =
        BaseStatusSignal.refreshAll(
            frFollowerMotor.getStatorCurrent(), frFollowerMotor.getSupplyCurrent());

    StatusCode blFollowerStatus =
        BaseStatusSignal.refreshAll(
            blFollowerMotor.getStatorCurrent(), blFollowerMotor.getSupplyCurrent());

    StatusCode brFollowerStatus =
        BaseStatusSignal.refreshAll(
            brFollowerMotor.getStatorCurrent(), brFollowerMotor.getSupplyCurrent());

    StatusCode encoderStatus =
        BaseStatusSignal.refreshAll(encoder.getAbsolutePosition(), encoder.getVelocity());

    inputs.flConnected = flLeaderDebounce.calculate(flLeaderStatus.isOK());
    inputs.frConnected = frFollowerDebounce.calculate(frFollowerStatus.isOK());
    inputs.blConnected = blFollowerDebounce.calculate(blFollowerStatus.isOK());
    inputs.brConnected = brFollowerDebounce.calculate(brFollowerStatus.isOK());
    inputs.encoderConnected = encoderDebounce.calculate(encoderStatus.isOK());

    // Update cached values with thread safety
    try {
      sensorLock.writeLock().lock();

      cachedShoulderVoltage = flLeaderMotor.getMotorVoltage().getValueAsDouble();
      cachedShoulderCurrent = flLeaderMotor.getSupplyCurrent().getValueAsDouble();
      cachedShoulderTemperature = flLeaderMotor.getDeviceTemp().getValueAsDouble();
      cachedShoulderPosition =
          flLeaderMotor.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
      cachedShoulderVelocity =
          flLeaderMotor.getVelocity().getValueAsDouble() * DEGREES_PER_ROTATION;
      cachedEncoderPosition = encoder.getAbsolutePosition().getValueAsDouble();
      cachedEncoderVelocity = encoder.getVelocity().getValueAsDouble();

      // Update inputs with the same values
      inputs.shoulderVoltage = cachedShoulderVoltage;
      inputs.shoulderCurrent = cachedShoulderCurrent;
      inputs.shoulderTemperature = cachedShoulderTemperature;
      inputs.shoulderPositionDegrees = cachedShoulderPosition;
      inputs.shoulderVelocityDegrees = cachedShoulderVelocity;
      inputs.encoderPosition = cachedEncoderPosition;
      inputs.encoderVelocity = cachedEncoderVelocity;
    } finally {
      sensorLock.writeLock().unlock();
    }
  }

  @Override
  public void setPercentage(double percentage) {
    flLeaderMotor.set(percentage);
  }

  @Override
  public Supplier<Double> getSignalSupplier(SignalType signalType) {
    return () -> {
      try {
        sensorLock.readLock().lock();
        switch (signalType) {
          case POSITION:
            return cachedShoulderPosition;
          case VELOCITY:
            return cachedShoulderVelocity;
          case VOLTAGE:
            return cachedShoulderVoltage;
          case CURRENT:
            return cachedShoulderCurrent;
          case TEMPERATURE:
            return cachedShoulderTemperature;
          case ENCODER_POSITION:
            return cachedEncoderPosition;
          case ENCODER_VELOCITY:
            return cachedEncoderVelocity;
          default:
            return 0.0;
        }
      } finally {
        sensorLock.readLock().unlock();
      }
    };
  }
}
