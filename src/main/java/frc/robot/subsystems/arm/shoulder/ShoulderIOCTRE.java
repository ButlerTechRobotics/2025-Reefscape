// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
 * CTRE-based implementation of the ShoulderIO interface for controlling a robot shoulder mechanism.
 * This implementation uses TalonFX motors and a CANcoder for position feedback. The shoulder
 * consists of four motors (back-right leader, back-left follower, front-right follower, front-left
 * follower) and an encoder for precise angular positioning.
 */
public class ShoulderIOCTRE implements ShoulderIO {
  /** The gear ratio between the motor and the shoulder mechanism */
  public static final double GEAR_RATIO = 135;

  /** CAN bus that the devices are located on */
  public static final CANBus kCANBus = new CANBus("CANivore");

  /** The back-right leader TalonFX motor controller (CAN ID: 16) */
  public final TalonFX brLeader = new TalonFX(23, kCANBus);
  /** The back-left follower TalonFX motor controller (CAN ID: 15) */
  public final TalonFX blFollower = new TalonFX(22, kCANBus);
  /** The front-right follower TalonFX motor controller (CAN ID: 14) */
  public final TalonFX frFollower = new TalonFX(21, kCANBus);
  /** The front-left follower TalonFX motor controller (CAN ID: 13) */
  public final TalonFX flFollower = new TalonFX(20, kCANBus);

  /** The CANcoder for position feedback (CAN ID: 17) */
  public final CANcoder encoder = new CANcoder(24, kCANBus);

  // Status signals for monitoring motor and encoder states
  private final StatusSignal<Angle> brLeaderPosition = brLeader.getPosition();
  private final StatusSignal<Angle> brLeaderRotorPosition = brLeader.getRotorPosition();
  private final StatusSignal<AngularVelocity> brLeaderVelocity = brLeader.getVelocity();
  private final StatusSignal<AngularVelocity> brLeaderRotorVelocity = brLeader.getRotorVelocity();
  private final StatusSignal<Voltage> brLeaderAppliedVolts = brLeader.getMotorVoltage();

  // Current signals for all motors
  private final StatusSignal<Current> brLeaderStatorCurrent = brLeader.getStatorCurrent();
  private final StatusSignal<Current> blFollowerStatorCurrent = blFollower.getStatorCurrent();
  private final StatusSignal<Current> frFollowerStatorCurrent = frFollower.getStatorCurrent();
  private final StatusSignal<Current> flFollowerStatorCurrent = flFollower.getStatorCurrent();

  private final StatusSignal<Current> brLeaderSupplyCurrent = brLeader.getSupplyCurrent();
  private final StatusSignal<Current> blFollowerSupplyCurrent = blFollower.getSupplyCurrent();
  private final StatusSignal<Current> frFollowerSupplyCurrent = frFollower.getSupplyCurrent();
  private final StatusSignal<Current> flFollowerSupplyCurrent = flFollower.getSupplyCurrent();

  private final StatusSignal<Angle> encoderPosition = encoder.getPosition();
  private final StatusSignal<AngularVelocity> encoderVelocity = encoder.getVelocity();

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer brLeaderDebounce = new Debouncer(0.5);
  private final Debouncer blFollowerDebounce = new Debouncer(0.5);
  private final Debouncer frFollowerDebounce = new Debouncer(0.5);
  private final Debouncer flFollowerDebounce = new Debouncer(0.5);
  private final Debouncer encoderDebounce = new Debouncer(0.5);

  /**
   * Constructs a new ShoulderIOCTRE instance and initializes all hardware components. This includes
   * configuring all four motors, setting up the follower relationships, and optimizing CAN bus
   * utilization for all devices.
   */
  public ShoulderIOCTRE() {
    // Set the CANcoder position to zero at startup
    // This makes it behave like a relative encoder instead of absolute
    encoder.setPosition(0.0);

    // Set up all three followers to mirror the leader
    blFollower.setControl(new Follower(brLeader.getDeviceID(), true));
    frFollower.setControl(new Follower(brLeader.getDeviceID(), false));
    flFollower.setControl(new Follower(brLeader.getDeviceID(), true));

    // Configure leader motor with appropriate settings
    TalonFXConfiguration config = createMotorConfiguration();
    for (int i = 0; i < 4; i++) {
      boolean statusOK = brLeader.getConfigurator().apply(config, 0.1) == StatusCode.OK;
      if (statusOK) break;
    }

    // Configure update frequencies for all status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50Hz update rate
        brLeaderPosition,
        brLeaderRotorPosition,
        brLeaderVelocity,
        brLeaderRotorVelocity,
        brLeaderAppliedVolts,
        brLeaderStatorCurrent,
        blFollowerStatorCurrent,
        frFollowerStatorCurrent,
        flFollowerStatorCurrent,
        brLeaderSupplyCurrent,
        blFollowerSupplyCurrent,
        frFollowerSupplyCurrent,
        flFollowerSupplyCurrent,
        encoderPosition,
        encoderVelocity);

    // Optimize CAN bus usage for all devices
    brLeader.optimizeBusUtilization(4, 0.1);
    blFollower.optimizeBusUtilization(4, 0.1);
    frFollower.optimizeBusUtilization(4, 0.1);
    flFollower.optimizeBusUtilization(4, 0.1);
    encoder.optimizeBusUtilization(4, 0.1);
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
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.MotionMagic.MotionMagicCruiseVelocity = 2.5;
    config.MotionMagic.MotionMagicAcceleration = 10;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.5;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

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
    // Hardware-specific PID values
    config.Slot0.kP = 110; // Proportional gain
    config.Slot0.kI = 0; // Integral gain
    config.Slot0.kD = 25; // Derivative gain
    config.Slot0.kS = 1; // Static friction compensation
    config.Slot0.kV = 0; // Velocity feedforward
    config.Slot0.kA = 0; // Acceleration feedforward
    config.Slot0.kG = 0; // Gravity feedforward
    return config;
  }
  /**
   * Updates the shoulder's input values with the latest sensor readings. This includes position,
   * velocity, voltage, and current measurements from all motors and the encoder, as well as
   * connection status for all devices.
   *
   * @param inputs The ShoulderIOInputs object to update with the latest values
   */
  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    // Refresh all sensor data
    StatusCode brLeaderStatus =
        BaseStatusSignal.refreshAll(
            brLeaderPosition,
            brLeaderRotorPosition,
            brLeaderVelocity,
            brLeaderRotorVelocity,
            brLeaderAppliedVolts,
            brLeaderStatorCurrent,
            brLeaderSupplyCurrent);

    StatusCode blFollowerStatus =
        BaseStatusSignal.refreshAll(blFollowerStatorCurrent, blFollowerSupplyCurrent);
    StatusCode frFollowerStatus =
        BaseStatusSignal.refreshAll(frFollowerStatorCurrent, frFollowerSupplyCurrent);
    StatusCode flFollowerStatus =
        BaseStatusSignal.refreshAll(flFollowerStatorCurrent, flFollowerSupplyCurrent);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    // Update connection status with debouncing
    inputs.brLeaderConnected = brLeaderDebounce.calculate(brLeaderStatus.isOK());
    inputs.blFollowerConnected = blFollowerDebounce.calculate(blFollowerStatus.isOK());
    inputs.frFollowerConnected = frFollowerDebounce.calculate(frFollowerStatus.isOK());
    inputs.flFollowerConnected = flFollowerDebounce.calculate(flFollowerStatus.isOK());
    inputs.encoderConnected = encoderDebounce.calculate(encoderStatus.isOK());

    // Update position and velocity measurements
    inputs.brLeaderPosition = brLeaderPosition.getValue();
    inputs.brLeaderRotorPosition = brLeaderRotorPosition.getValue();
    inputs.brLeaderVelocity = brLeaderVelocity.getValue();
    inputs.brLeaderRotorVelocity = brLeaderRotorVelocity.getValue();

    inputs.encoderPosition = encoderPosition.getValue();
    inputs.encoderVelocity = encoderVelocity.getValue();

    // Update voltage and current measurements
    inputs.appliedVoltage = brLeaderAppliedVolts.getValue();

    // Update stator currents for all motors
    inputs.brLeaderStatorCurrent = brLeaderStatorCurrent.getValue();
    inputs.blFollowerStatorCurrent = blFollowerStatorCurrent.getValue();
    inputs.frFollowerStatorCurrent = frFollowerStatorCurrent.getValue();
    inputs.flFollowerStatorCurrent = flFollowerStatorCurrent.getValue();

    // Update supply currents for all motors
    inputs.brLeaderSupplyCurrent = brLeaderSupplyCurrent.getValue();
    inputs.blFollowerSupplyCurrent = blFollowerSupplyCurrent.getValue();
    inputs.frFollowerSupplyCurrent = frFollowerSupplyCurrent.getValue();
    inputs.flFollowerSupplyCurrent = flFollowerSupplyCurrent.getValue();

    // Calculate shoulder angle using encoder position
    inputs.shoulderAngle = inputs.encoderPosition;
  }

  /**
   * Sets the desired angle for the shoulder to move to. This should be based off encoder rotations
   * to shoulder.
   *
   * @param angle The target angle for the shoulder mechanism
   */
  @Override
  public void setPosition(Angle angle) {
    // Convert desired angle to encoder rotations
    brLeader.setControl(new MotionMagicTorqueCurrentFOC(angle));
  }

  /**
   * Runs the shoulder in open loop at the specified voltage. This method is used for manual control
   * of the shoulder mechanism.
   *
   * @param volts The voltage to run the shoulder at
   */
  @Override
  public void runVolts(double volts) {
    brLeader.setControl(new VoltageOut(volts).withUpdateFreqHz(0));
  }

  /**
   * Stops all shoulder movement by stopping the leader motor. All followers will also stop due to
   * the follower relationship.
   */
  @Override
  public void stop() {
    brLeader.stopMotor();
  }

  /**
   * Sets the neutral mode (brake or coast) for all shoulder motors.
   *
   * <p>This method runs in a separate thread to prevent blocking the main control loop, as changing
   * neutral mode can take a non-trivial amount of time on the CAN bus. The setting is only applied
   * to the leader motor, as all followers will automatically inherit this setting through their
   * follower relationship.
   *
   * <p>In brake mode ({@code enabled=true}), motors will actively resist external movement when not
   * powered, which helps maintain position but generates heat. In coast mode ({@code
   * enabled=false}), motors spin freely when not powered, allowing for manual positioning but
   * potentially drifting due to gravity.
   *
   * @param enabled true to enable brake mode, false for coast mode
   */
  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () ->
                brLeader.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
