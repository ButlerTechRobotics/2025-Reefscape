// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

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
 * implementation uses TalonFX motors and CANcoders for position feedback. The arm consists of joint
 * and extension mechanisms, each with a leader motor, follower motor, and encoder for precise
 * positioning.
 */
public class ArmIOCTRE implements ArmIO {
  // Gear ratios
  public static final double JOINT_GEAR_RATIO = 150.0;
  public static final double EXTENSION_GEAR_RATIO = 2.0;

  // Joint motors and encoder
  public final TalonFX jointLeader = new TalonFX(20);
  public final TalonFX jointFollower = new TalonFX(21);
  public final CANcoder jointEncoder = new CANcoder(22);

  // Extension motors and encoder
  public final TalonFX extensionLeader = new TalonFX(30);
  public final TalonFX extensionFollower = new TalonFX(31);
  public final CANcoder extensionEncoder = new CANcoder(32);

  // Joint status signals
  private final StatusSignal<Angle> jointLeaderPosition = jointLeader.getPosition();
  private final StatusSignal<Angle> jointLeaderRotorPosition = jointLeader.getRotorPosition();
  private final StatusSignal<AngularVelocity> jointLeaderVelocity = jointLeader.getVelocity();
  private final StatusSignal<AngularVelocity> jointLeaderRotorVelocity =
      jointLeader.getRotorVelocity();
  private final StatusSignal<Voltage> jointLeaderAppliedVolts = jointLeader.getMotorVoltage();
  private final StatusSignal<Current> jointLeaderStatorCurrent = jointLeader.getStatorCurrent();
  private final StatusSignal<Current> jointFollowerStatorCurrent = jointFollower.getStatorCurrent();
  private final StatusSignal<Current> jointLeaderSupplyCurrent = jointLeader.getSupplyCurrent();
  private final StatusSignal<Current> jointFollowerSupplyCurrent = jointFollower.getSupplyCurrent();
  private final StatusSignal<Angle> jointEncoderPosition = jointEncoder.getPosition();
  private final StatusSignal<AngularVelocity> jointEncoderVelocity = jointEncoder.getVelocity();

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
  private final Debouncer jointLeaderDebounce = new Debouncer(0.5);
  private final Debouncer jointFollowerDebounce = new Debouncer(0.5);
  private final Debouncer jointEncoderDebounce = new Debouncer(0.5);
  private final Debouncer extensionLeaderDebounce = new Debouncer(0.5);
  private final Debouncer extensionFollowerDebounce = new Debouncer(0.5);
  private final Debouncer extensionEncoderDebounce = new Debouncer(0.5);

  // Extension mechanism constants
  protected final Distance extensionRadius = Inches.of(2);

  /** Constructs a new ArmIOCTRE instance and initializes all hardware components. */
  public ArmIOCTRE() {
    // Set up joint follower
    jointFollower.setControl(new Follower(jointLeader.getDeviceID(), false));

    // Set up extension follower
    extensionFollower.setControl(new Follower(extensionLeader.getDeviceID(), false));

    // Configure motors
    jointLeader.getConfigurator().apply(createJointConfiguration());
    extensionLeader.getConfigurator().apply(createExtensionConfiguration());

    // Configure update frequencies for joint signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        jointLeaderPosition,
        jointLeaderRotorPosition,
        jointLeaderVelocity,
        jointLeaderRotorVelocity,
        jointLeaderAppliedVolts,
        jointLeaderStatorCurrent,
        jointFollowerStatorCurrent,
        jointLeaderSupplyCurrent,
        jointFollowerSupplyCurrent,
        jointEncoderPosition,
        jointEncoderVelocity);

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
    jointLeader.optimizeBusUtilization(4, 0.1);
    jointFollower.optimizeBusUtilization(4, 0.1);
    jointEncoder.optimizeBusUtilization(4, 0.1);
    extensionLeader.optimizeBusUtilization(4, 0.1);
    extensionFollower.optimizeBusUtilization(4, 0.1);
    extensionEncoder.optimizeBusUtilization(4, 0.1);
  }

  /** Creates the joint motor configuration. */
  private TalonFXConfiguration createJointConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 620;
    config.Slot0.kI = 0;
    config.Slot0.kD = 11;
    config.Slot0.kS = 0.08;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kG = 0.0001;
    config.Feedback.withRemoteCANcoder(jointEncoder);
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
    // Update joint sensors
    StatusCode jointLeaderStatus =
        BaseStatusSignal.refreshAll(
            jointLeaderPosition,
            jointLeaderRotorPosition,
            jointLeaderVelocity,
            jointLeaderRotorVelocity,
            jointLeaderAppliedVolts,
            jointLeaderStatorCurrent,
            jointLeaderSupplyCurrent);

    StatusCode jointFollowerStatus =
        BaseStatusSignal.refreshAll(jointFollowerStatorCurrent, jointFollowerSupplyCurrent);

    StatusCode jointEncoderStatus =
        BaseStatusSignal.refreshAll(jointEncoderPosition, jointEncoderVelocity);

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

    // Update connection statuses
    inputs.jointLeaderConnected = jointLeaderDebounce.calculate(jointLeaderStatus.isOK());
    inputs.jointFollowerConnected = jointFollowerDebounce.calculate(jointFollowerStatus.isOK());
    inputs.jointEncoderConnected = jointEncoderDebounce.calculate(jointEncoderStatus.isOK());
    inputs.extensionLeaderConnected =
        extensionLeaderDebounce.calculate(extensionLeaderStatus.isOK());
    inputs.extensionFollowerConnected =
        extensionFollowerDebounce.calculate(extensionFollowerStatus.isOK());
    inputs.extensionEncoderConnected =
        extensionEncoderDebounce.calculate(extensionEncoderStatus.isOK());

    // Update joint measurements
    inputs.jointLeaderPosition = jointLeaderPosition.getValue();
    inputs.jointLeaderRotorPosition = jointLeaderRotorPosition.getValue();
    inputs.jointLeaderVelocity = jointLeaderVelocity.getValue();
    inputs.jointLeaderRotorVelocity = jointLeaderRotorVelocity.getValue();
    inputs.jointEncoderPosition = jointEncoderPosition.getValue();
    inputs.jointEncoderVelocity = jointEncoderVelocity.getValue();
    inputs.jointAppliedVoltage = jointLeaderAppliedVolts.getValue();
    inputs.jointLeaderStatorCurrent = jointLeaderStatorCurrent.getValue();
    inputs.jointFollowerStatorCurrent = jointFollowerStatorCurrent.getValue();
    inputs.jointLeaderSupplyCurrent = jointLeaderSupplyCurrent.getValue();
    inputs.jointFollowerSupplyCurrent = jointFollowerSupplyCurrent.getValue();
    inputs.jointAngle = inputs.jointEncoderPosition;

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
   * Sets the desired angle for the arm joint and the desired distance for the arm extension.
   *
   * @param angle The target joint angle
   * @param distance The target extension distance
   */
  @Override
  public void setPosition(Angle angle, Distance distance) {
    jointLeader.setControl(new PositionVoltage(angle));
    extensionLeader.setControl(
        new PositionVoltage(Conversions.metersToRotations(distance, 1, extensionRadius)));
  }

  /** Stops all arm movement. */
  @Override
  public void stop() {
    jointLeader.stopMotor();
    extensionLeader.stopMotor();
  }
}
