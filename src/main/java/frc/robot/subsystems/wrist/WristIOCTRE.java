// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * CTRE-based implementation of the WristIO interface for controlling a robot wrist mechanism. This
 * implementation uses TalonFX motors and a CANcoder for position feedback. The wrist consists of a
 * leader motor, a follower motor, and an encoder for precise angular positioning.
 */
public class WristIOCTRE implements WristIO {
  /** The gear ratio between the motor and the wrist mechanism */
  public static final double GEAR_RATIO = ((50 / 12) * (42 / 16) * (42 / 16)); // (32 + (2 / 3))

  /** CAN bus that the devices are located on */
  public static final CANBus kCANBus = new CANBus("CANivore");

  /** The leader TalonFX motor controller (CAN ID: 40) */
  public final TalonFX leader = new TalonFX(40, kCANBus);

  /** The CANcoder for position feedback (CAN ID: 41) */
  public final CANcoder encoder = new CANcoder(41, kCANBus);

  // Status signals for monitoring motor and encoder states
  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<Angle> leaderRotorPosition = leader.getRotorPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<AngularVelocity> leaderRotorVelocity = leader.getRotorVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Angle> encoderPosition = encoder.getPosition();
  private final StatusSignal<AngularVelocity> encoderVelocity = encoder.getVelocity();

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer encoderDebounce = new Debouncer(0.5);

  // Current control slot
  private int activeSlot = 0;

  /**
   * Constructs a new WristIOCTRE instance and initializes all hardware components. This includes
   * configuring the motor, and optimizing CAN bus utilization for all devices.
   */
  public WristIOCTRE() {
    // Set the CANcoder position to zero at startup
    // This makes it behave like a relative encoder instead of absolute
    encoder.setPosition(0.0);

    // Configure motor with settings
    TalonFXConfiguration config = createMotorConfiguration();
    leader.getConfigurator().apply(config);

    // Configure update frequencies for all status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50Hz update rate
        leaderPosition,
        leaderRotorPosition,
        leaderVelocity,
        leaderRotorVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        encoderPosition,
        encoderVelocity);

    // Optimize CAN bus usage for all devices
    leader.optimizeBusUtilization(50, 0.1);
    encoder.optimizeBusUtilization(50, 0.1);
  }

  /**
   * Creates the motor configuration with appropriate settings. Sets up neutral mode, PID gains, and
   * feedback device configuration.
   *
   * @return The configured TalonFXConfiguration object
   */
  private TalonFXConfiguration createMotorConfiguration() {
    var config = new TalonFXConfiguration();
    // Set motor to coast when stopped
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotionMagic.MotionMagicCruiseVelocity = 6;
    // config.MotionMagic.MotionMagicAcceleration = 10;
    config.MotionMagic.MotionMagicAcceleration = 7;

    // Apply PID and feedforward values from protected method
    configPID(config);

    // Use the CANcoder as the remote feedback device
    config.Feedback.withRemoteCANcoder(encoder);
    return config;
  }
  /**
   * Configures PID and feedforward gains for the hardware implementation. This method can be
   * overridden in simulation to provide different tuning.
   *
   * @param config The TalonFXConfiguration to apply PID values to
   * @return The updated configuration with PID values applied
   */
  protected TalonFXConfiguration configPID(TalonFXConfiguration config) {

    // Configure Slot 0 (No game piece)
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.kP = 220; // Proportional gain
    config.Slot0.kI = 0; // Integral gain
    config.Slot0.kD = 8; // Derivative gain
    config.Slot0.kS = 2; // Static friction compensation
    config.Slot0.kV = 0; // Velocity feedforward
    config.Slot0.kA = 0; // Acceleration feedforward
    config.Slot0.kG = 12; // Gravity feedforward

    // Configure Slot 1 (With game piece - adjusted for added mass)
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot1.kP = 220; // Higher P gain for increased load
    config.Slot1.kI = 0; // Integral gain
    config.Slot1.kD = 8; // Higher D gain for better damping
    config.Slot1.kS = 2; // Higher static friction compensation
    config.Slot1.kV = 0; // Velocity feedforward
    config.Slot1.kA = 0; // Acceleration feedforward
    config.Slot1.kG = 15; // Higher gravity compensation for added mass

    return config;
  }

  /**
   * Updates the wrist's input values with the latest sensor readings. This includes position,
   * velocity, voltage, and current measurements from both motors and the encoder, as well as
   * connection status for all devices.
   *
   * @param inputs The WristIOInputs object to update with the latest values
   */
  @Override
  public void updateInputs(WristIOInputs inputs) {
    // Refresh all sensor data
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderRotorPosition,
            leaderVelocity,
            leaderRotorVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    // Update connection status with debouncing
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.encoderConnected = encoderDebounce.calculate(encoderStatus.isOK());

    // Update position and velocity measurements
    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderRotorPosition = leaderRotorPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderRotorVelocity = leaderRotorVelocity.getValue();

    inputs.encoderPosition = encoderPosition.getValue();
    inputs.encoderVelocity = encoderVelocity.getValue();

    // Update voltage and current measurements
    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();

    // Calculate wrist angle using encoder position
    inputs.wristAngle = inputs.encoderPosition;

    // Record the active control slot
    inputs.activeControlSlot = activeSlot;
  }

  /**
   * Sets the desired voltage for the wrist to move with.
   *
   * @param volts The target voltage for the wrist mechanism
   */
  @Override
  public void setVoltage(Voltage volts) {
    // Convert desired angle to encoder rotations
    leader.setControl(new VoltageOut(volts));
  }

  /**
   * Sets the desired angle for the wrist to move to. This should be based off encoder rotations to
   * wrist.
   *
   * @param angle The target angle for the wrist mechanism
   */
  @Override
  public void setPosition(Angle angle) {
    // Convert desired angle to encoder rotations
    leader.setControl(new MotionMagicTorqueCurrentFOC(angle).withSlot(activeSlot));
  }

  /**
   * Sets which control slot to use for the wrist motor.
   *
   * @param slot The slot number to use (0 = no game piece, 1 = with game piece)
   */
  @Override
  public void setControlSlot(int slot) {
    if (slot != activeSlot) {
      activeSlot = slot;
    }
  }

  /**
   * Sets the encoder position to a specific value.
   *
   * @param position The position to set the encoder to
   */
  @Override
  public void setEncoderPosition(Angle position) {
    encoder.setPosition(position);
  }

  /**
   * Stops all wrist movement by stopping the leader motor. The follower will also stop due to the
   * follower relationship.
   */
  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> leader.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
