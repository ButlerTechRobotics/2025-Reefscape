// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.extension;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Conversions;

/**
 * CTRE-based implementation of the ExtensionIO interface for controlling an extension mechanism.
 * This implementation uses TalonFX motors and a CANcoder for position feedback. The extension
 * consists of a leader motor, a follower motor, and an encoder for precise positioning.
 */
public class ExtensionIOCTRE implements ExtensionIO {
  /** The gear ratio between the motor and the extension mechanism */
  public static final double GEAR_RATIO = 90.0;

  /** CAN bus that the devices are located on */
  public static final CANBus kCANBus = new CANBus("CANivore");

  /** The leader TalonFX motor controller (CAN ID: 30) */
  public final TalonFX leader = new TalonFX(30, kCANBus);
  /** The follower TalonFX motor controller (CAN ID: 31) */
  public final TalonFX follower = new TalonFX(31, kCANBus);

  // Status signals for monitoring motor and encoder states
  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<Angle> leaderRotorPosition = leader.getRotorPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<AngularVelocity> leaderRotorVelocity = leader.getRotorVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer followerDebounce = new Debouncer(0.5);

  /**
   * The radius of the extension pulley/drum, used for converting between rotations and linear
   * distance
   */
  protected final Distance extensionRadius = Inches.of(0.5);

  // Current control slot
  private int activeSlot = 0;

  /**
   * Constructs a new ExtensionIOCTRE instance and initializes all hardware components. This
   * includes configuring both motors, setting up the follower relationship, and optimizing CAN bus
   * utilization for all devices.
   */
  public ExtensionIOCTRE() {
    leader.setPosition(0);
    // Set up follower to mirror leader
    follower.setControl(new Follower(leader.getDeviceID(), false));

    // Configure both motors with identical settings
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
        followerStatorCurrent,
        leaderSupplyCurrent,
        followerSupplyCurrent);

    // Optimize CAN bus usage for all devices
    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);
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
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotionMagic.MotionMagicCruiseVelocity = 90; // 60 3/14
    config.MotionMagic.MotionMagicAcceleration = 70; // 40 3/14
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 25;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    // Apply PID and feedforward values from protected method
    configPID(config);
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
    // Hardware-specific PID values

    // Configure Slot 0 (Horizontal)
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.kP = 90; // Proportional gain
    config.Slot0.kI = 0; // Integral gain
    config.Slot0.kD = 0; // Derivative gain
    config.Slot0.kS = 1; // Static friction compensation
    config.Slot0.kV = 0; // Velocity feedforward
    config.Slot0.kA = 0; // Acceleration feedforward
    config.Slot0.kG = 6; // Gravity feedforward

    // Configure Slot 1 (Vertical)
    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot1.kP = 90; // Higher P gain for increased load
    config.Slot1.kI = 0; // Integral gain
    config.Slot1.kD = 0; // Higher D gain for better damping
    config.Slot1.kS = 1; // Higher static friction compensation
    config.Slot1.kV = 0; // Velocity feedforward
    config.Slot1.kA = 0; // Acceleration feedforward
    config.Slot1.kG = 6; // Higher gravity compensation for added mass

    return config;
  }

  /**
   * Updates the extension's input values with the latest sensor readings. This includes position,
   * velocity, voltage, and current measurements from both motors and the encoder, as well as
   * connection status for all devices.
   *
   * @param inputs The ExtensionIOInputs object to update with the latest values
   */
  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
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

    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(followerStatorCurrent, followerSupplyCurrent);

    // Update connection status with debouncing
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());

    // Update position and velocity measurements
    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderRotorPosition = leaderRotorPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderRotorVelocity = leaderRotorVelocity.getValue();

    // Update voltage and current measurements
    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    // Calculate actual extension distance using encoder position
    // Note: Using gear ratio of 1 since encoder rotations match extension movement
    inputs.extensionDistance =
        Conversions.rotationsToMeters(inputs.leaderPosition, 1, extensionRadius);

    // Record the active control slot
    inputs.activeControlSlot = activeSlot;
  }

  /**
   * Sets the desired distance for the extension to move to. Converts the desired linear distance to
   * encoder rotations and applies position control.
   *
   * @param distance The target distance for the extension
   */
  @Override
  public void setDistance(Distance distance) {
    // Convert desired distance to rotations and set position control
    leader.setControl(
        new MotionMagicTorqueCurrentFOC(Conversions.metersToRotations(distance, 1, extensionRadius))
            .withSlot(activeSlot));
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
   * Stops all extension movement by stopping the leader motor. The follower will also stop due to
   * the follower relationship.
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
