// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Conversions;

/**
 * CTRE-based implementation of the ArmIO interface for controlling a robot arm mechanism. This
 * implementation uses TalonFX motors and CANcoders for position feedback. The arm consists of
 * shoulder and extension mechanisms, each with a leader motor, follower motor, and encoder for
 * precise positioning.
 */
public class ArmIOCTRE implements ArmIO {
  // Gear ratios
  public static final double SHOULDER_GEAR_RATIO = 150.0;
  public static final double WRIST_GEAR_RATIO = 150.0;
  public static final double EXTENSION_GEAR_RATIO = 2.0;

  // Shoulder motors and encoder
  public final TalonFX shoulderLeader = new TalonFX(20);
  public final TalonFX shoulderFollower = new TalonFX(21);
  public final CANcoder shoulderEncoder = new CANcoder(22);

  // Wrist motor and encoder
  public final TalonFX wristLeader = new TalonFX(30);
  public final CANcoder wristEncoder = new CANcoder(31);

  // Extension motors and encoder
  public final TalonFX extensionLeader = new TalonFX(40);
  public final TalonFX extensionFollower = new TalonFX(41);
  public final CANcoder extensionEncoder = new CANcoder(42);

  // Shoulder status signals
  private final StatusSignal<Angle> shoulderLeaderPosition = shoulderLeader.getPosition();
  private final StatusSignal<Angle> shoulderLeaderRotorPosition = shoulderLeader.getRotorPosition();
  private final StatusSignal<AngularVelocity> shoulderLeaderVelocity = shoulderLeader.getVelocity();
  private final StatusSignal<AngularVelocity> shoulderLeaderRotorVelocity =
      shoulderLeader.getRotorVelocity();
  private final StatusSignal<Voltage> shoulderLeaderAppliedVolts = shoulderLeader.getMotorVoltage();
  private final StatusSignal<Current> shoulderLeaderStatorCurrent =
      shoulderLeader.getStatorCurrent();
  private final StatusSignal<Current> shoulderFollowerStatorCurrent =
      shoulderFollower.getStatorCurrent();
  private final StatusSignal<Current> shoulderLeaderSupplyCurrent =
      shoulderLeader.getSupplyCurrent();
  private final StatusSignal<Current> shoulderFollowerSupplyCurrent =
      shoulderFollower.getSupplyCurrent();
  private final StatusSignal<Angle> shoulderEncoderPosition = shoulderEncoder.getPosition();
  private final StatusSignal<AngularVelocity> shoulderEncoderVelocity =
      shoulderEncoder.getVelocity();

  // Wrist status signals
  private final StatusSignal<Angle> wristLeaderPosition = wristLeader.getPosition();
  private final StatusSignal<Angle> wristLeaderRotorPosition = wristLeader.getRotorPosition();
  private final StatusSignal<AngularVelocity> wristLeaderVelocity = wristLeader.getVelocity();
  private final StatusSignal<AngularVelocity> wristLeaderRotorVelocity =
      wristLeader.getRotorVelocity();
  private final StatusSignal<Voltage> wristLeaderAppliedVolts = wristLeader.getMotorVoltage();
  private final StatusSignal<Current> wristLeaderStatorCurrent = wristLeader.getStatorCurrent();
  private final StatusSignal<Current> wristLeaderSupplyCurrent = wristLeader.getSupplyCurrent();
  private final StatusSignal<Angle> wristEncoderPosition = wristEncoder.getPosition();
  private final StatusSignal<AngularVelocity> wristEncoderVelocity = wristEncoder.getVelocity();

  // Extension status signals
  private final StatusSignal<Angle> extensionLeaderPosition = extensionLeader.getPosition();
  private final StatusSignal<Angle> extensionLeaderRotorPosition =
      extensionLeader.getRotorPosition();
  private final StatusSignal<AngularVelocity> extensionLeaderVelocity =
      extensionLeader.getVelocity();
  private final StatusSignal<AngularVelocity> extensionLeaderRotorVelocity =
      extensionLeader.getRotorVelocity();
  private final StatusSignal<Voltage> extensionLeaderAppliedVolts =
      extensionLeader.getMotorVoltage();
  private final StatusSignal<Current> extensionLeaderStatorCurrent =
      extensionLeader.getStatorCurrent();
  private final StatusSignal<Current> extensionFollowerStatorCurrent =
      extensionFollower.getStatorCurrent();
  private final StatusSignal<Current> extensionLeaderSupplyCurrent =
      extensionLeader.getSupplyCurrent();
  private final StatusSignal<Current> extensionFollowerSupplyCurrent =
      extensionFollower.getSupplyCurrent();
  private final StatusSignal<Angle> extensionEncoderPosition = extensionEncoder.getPosition();
  private final StatusSignal<AngularVelocity> extensionEncoderVelocity =
      extensionEncoder.getVelocity();

  // Connection debouncers
  private final Debouncer shoulderLeaderDebounce = new Debouncer(0.5);
  private final Debouncer shoulderFollowerDebounce = new Debouncer(0.5);
  private final Debouncer shoulderEncoderDebounce = new Debouncer(0.5);
  private final Debouncer wristLeaderDebounce = new Debouncer(0.5);
  private final Debouncer wristEncoderDebounce = new Debouncer(0.5);
  private final Debouncer extensionLeaderDebounce = new Debouncer(0.5);
  private final Debouncer extensionFollowerDebounce = new Debouncer(0.5);
  private final Debouncer extensionEncoderDebounce = new Debouncer(0.5);

  // Extension mechanism constants
  protected final Distance extensionRadius = Inches.of(2);

  /** Constructs a new ArmIOCTRE instance and initializes all hardware components. */
  public ArmIOCTRE() {
    // Set up shoulder follower
    shoulderFollower.setControl(new Follower(shoulderLeader.getDeviceID(), false));

    // Set up extension follower
    extensionFollower.setControl(new Follower(extensionLeader.getDeviceID(), false));

    // Configure motors
    shoulderLeader.getConfigurator().apply(createShoulderConfiguration());
    wristLeader.getConfigurator().apply(createWristConfiguration());
    extensionLeader.getConfigurator().apply(createExtensionConfiguration());

    // Configure update frequencies for shoulder signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shoulderLeaderPosition,
        shoulderLeaderRotorPosition,
        shoulderLeaderVelocity,
        shoulderLeaderRotorVelocity,
        shoulderLeaderAppliedVolts,
        shoulderLeaderStatorCurrent,
        shoulderFollowerStatorCurrent,
        shoulderLeaderSupplyCurrent,
        shoulderFollowerSupplyCurrent,
        shoulderEncoderPosition,
        shoulderEncoderVelocity);

    // Configure update frequencies for wrist signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristLeaderPosition,
        wristLeaderRotorPosition,
        wristLeaderVelocity,
        wristLeaderRotorVelocity,
        wristLeaderAppliedVolts,
        wristLeaderStatorCurrent,
        wristLeaderSupplyCurrent,
        wristEncoderPosition,
        wristEncoderVelocity);

    // Configure update frequencies for extension signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionLeaderPosition,
        extensionLeaderRotorPosition,
        extensionLeaderVelocity,
        extensionLeaderRotorVelocity,
        extensionLeaderAppliedVolts,
        extensionLeaderStatorCurrent,
        extensionFollowerStatorCurrent,
        extensionLeaderSupplyCurrent,
        extensionFollowerSupplyCurrent,
        extensionEncoderPosition,
        extensionEncoderVelocity);

    // Optimize CAN bus usage
    shoulderLeader.optimizeBusUtilization(4, 0.1);
    shoulderFollower.optimizeBusUtilization(4, 0.1);
    shoulderEncoder.optimizeBusUtilization(4, 0.1);
    wristLeader.optimizeBusUtilization(4, 0.1);
    wristEncoder.optimizeBusUtilization(4, 0.1);
    extensionLeader.optimizeBusUtilization(4, 0.1);
    extensionFollower.optimizeBusUtilization(4, 0.1);
    extensionEncoder.optimizeBusUtilization(4, 0.1);
  }

  /** Creates the shoulder motor configuration. */
  private TalonFXConfiguration createShoulderConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 11;
    config.Slot0.kS = 0.08;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kG = 0.0001;
    config.Feedback.withRemoteCANcoder(shoulderEncoder);
    return config;
  }

  /** Creates the wrist motor configuration. */
  private TalonFXConfiguration createWristConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 620;
    config.Slot0.kI = 0;
    config.Slot0.kD = 11;
    config.Slot0.kS = 0.08;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kG = 0.0001;
    config.Feedback.withRemoteCANcoder(wristEncoder);
    return config;
  }

  /** Creates the extension motor configuration. */
  private TalonFXConfiguration createExtensionConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 24;
    config.Slot0.kI = 0;
    config.Slot0.kD = 1.6;
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kG = 0.7297;
    config.Feedback.withRemoteCANcoder(extensionEncoder);
    return config;
  }

  /** Updates all arm input values with the latest sensor readings. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update shoulder sensors
    StatusCode shoulderLeaderStatus =
        BaseStatusSignal.refreshAll(
            shoulderLeaderPosition,
            shoulderLeaderRotorPosition,
            shoulderLeaderVelocity,
            shoulderLeaderRotorVelocity,
            shoulderLeaderAppliedVolts,
            shoulderLeaderStatorCurrent,
            shoulderLeaderSupplyCurrent);

    StatusCode shoulderFollowerStatus =
        BaseStatusSignal.refreshAll(shoulderFollowerStatorCurrent, shoulderFollowerSupplyCurrent);

    StatusCode shoulderEncoderStatus =
        BaseStatusSignal.refreshAll(shoulderEncoderPosition, shoulderEncoderVelocity);

    // Update extension sensors
    StatusCode extensionLeaderStatus =
        BaseStatusSignal.refreshAll(
            extensionLeaderPosition,
            extensionLeaderRotorPosition,
            extensionLeaderVelocity,
            extensionLeaderRotorVelocity,
            extensionLeaderAppliedVolts,
            extensionLeaderStatorCurrent,
            extensionLeaderSupplyCurrent);

    StatusCode extensionFollowerStatus =
        BaseStatusSignal.refreshAll(extensionFollowerStatorCurrent, extensionFollowerSupplyCurrent);

    StatusCode extensionEncoderStatus =
        BaseStatusSignal.refreshAll(extensionEncoderPosition, extensionEncoderVelocity);

    // Update wrist sensors
    StatusCode wristLeaderStatus =
        BaseStatusSignal.refreshAll(
            wristLeaderPosition,
            wristLeaderRotorPosition,
            wristLeaderVelocity,
            wristLeaderRotorVelocity,
            wristLeaderAppliedVolts,
            wristLeaderStatorCurrent,
            wristLeaderSupplyCurrent);

    StatusCode wristEncoderStatus =
        BaseStatusSignal.refreshAll(wristEncoderPosition, wristEncoderVelocity);

    // Update connection statuses
    inputs.shoulderLeaderConnected = shoulderLeaderDebounce.calculate(shoulderLeaderStatus.isOK());
    inputs.shoulderFollowerConnected =
        shoulderFollowerDebounce.calculate(shoulderFollowerStatus.isOK());
    inputs.shoulderEncoderConnected =
        shoulderEncoderDebounce.calculate(shoulderEncoderStatus.isOK());
    inputs.wristLeaderConnected = wristLeaderDebounce.calculate(wristLeaderStatus.isOK());
    inputs.wristEncoderConnected = wristEncoderDebounce.calculate(wristEncoderStatus.isOK());
    inputs.extensionLeaderConnected =
        extensionLeaderDebounce.calculate(extensionLeaderStatus.isOK());
    inputs.extensionFollowerConnected =
        extensionFollowerDebounce.calculate(extensionFollowerStatus.isOK());
    inputs.extensionEncoderConnected =
        extensionEncoderDebounce.calculate(extensionEncoderStatus.isOK());

    // Update shoulder measurements
    inputs.shoulderLeaderPosition = shoulderLeaderPosition.getValue();
    inputs.shoulderLeaderRotorPosition = shoulderLeaderRotorPosition.getValue();
    inputs.shoulderLeaderVelocity = shoulderLeaderVelocity.getValue();
    inputs.shoulderLeaderRotorVelocity = shoulderLeaderRotorVelocity.getValue();
    inputs.shoulderEncoderPosition = shoulderEncoderPosition.getValue();
    inputs.shoulderEncoderVelocity = shoulderEncoderVelocity.getValue();
    inputs.shoulderAppliedVoltage = shoulderLeaderAppliedVolts.getValue();
    inputs.shoulderLeaderStatorCurrent = shoulderLeaderStatorCurrent.getValue();
    inputs.shoulderFollowerStatorCurrent = shoulderFollowerStatorCurrent.getValue();
    inputs.shoulderLeaderSupplyCurrent = shoulderLeaderSupplyCurrent.getValue();
    inputs.shoulderFollowerSupplyCurrent = shoulderFollowerSupplyCurrent.getValue();
    inputs.shoulderAngle = inputs.shoulderEncoderPosition;

    // Update wrist measurements
    inputs.wristLeaderPosition = wristLeaderPosition.getValue();
    inputs.wristLeaderRotorPosition = wristLeaderRotorPosition.getValue();
    inputs.wristLeaderVelocity = wristLeaderVelocity.getValue();
    inputs.wristLeaderRotorVelocity = wristLeaderRotorVelocity.getValue();
    inputs.wristEncoderPosition = wristEncoderPosition.getValue();
    inputs.wristEncoderVelocity = wristEncoderVelocity.getValue();
    inputs.wristAppliedVoltage = wristLeaderAppliedVolts.getValue();
    inputs.wristLeaderStatorCurrent = wristLeaderStatorCurrent.getValue();
    inputs.wristLeaderSupplyCurrent = wristLeaderSupplyCurrent.getValue();
    inputs.wristAngle = inputs.wristEncoderPosition;

    // Update extension measurements
    inputs.extensionLeaderPosition = extensionLeaderPosition.getValue();
    inputs.extensionLeaderRotorPosition = extensionLeaderRotorPosition.getValue();
    inputs.extensionLeaderVelocity = extensionLeaderVelocity.getValue();
    inputs.extensionLeaderRotorVelocity = extensionLeaderRotorVelocity.getValue();
    inputs.extensionEncoderPosition = extensionEncoderPosition.getValue();
    inputs.extensionEncoderVelocity = extensionEncoderVelocity.getValue();
    inputs.extensionAppliedVoltage = extensionLeaderAppliedVolts.getValue();
    inputs.extensionLeaderStatorCurrent = extensionLeaderStatorCurrent.getValue();
    inputs.extensionFollowerStatorCurrent = extensionFollowerStatorCurrent.getValue();
    inputs.extensionLeaderSupplyCurrent = extensionLeaderSupplyCurrent.getValue();
    inputs.extensionFollowerSupplyCurrent = extensionFollowerSupplyCurrent.getValue();
    inputs.extensionDistance =
        Conversions.rotationsToMeters(inputs.extensionEncoderPosition, 1, extensionRadius);
  }

  /**
   * Runs the arm in closed-loop position mode to the specified angles and extension.
   *
   * @param shoulderAngle The target shoulder angle
   * @param wristAngle The target wrist angle
   * @param extensionDistance The target extension distance
   */
  @Override
  public void setPosition(Angle shoulderAngle, Angle wristAngle, Distance extensionDistance) {
    shoulderLeader.setControl(new PositionVoltage(shoulderAngle));
    wristLeader.setControl(new PositionVoltage(wristAngle));
    extensionLeader.setControl(
        new PositionVoltage(Conversions.inchesToRotations(extensionDistance, 1, extensionRadius)));
  }

  /** Stops all arm movement. */
  @Override
  public void stop() {
    shoulderLeader.stopMotor();
    wristLeader.stopMotor();
    extensionLeader.stopMotor();
  }
}
