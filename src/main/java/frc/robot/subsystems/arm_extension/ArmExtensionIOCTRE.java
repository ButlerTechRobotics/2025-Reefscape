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

package frc.robot.subsystems.arm_extension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ArmExtensionIOCTRE implements ArmExtensionIO {
  // Conversion factor from motor rotations to linear distance
  // For example, if using a chain/pulley system with a 2-inch diameter pulley:
  // One rotation = 2π * (2 inches / 2) = 6.28 inches of linear travel
  public static final double DRUM_CIRCUMFERENCE_METERS = 0.1016; // 4 inches in meters
  public static final double GEAR_RATIO = 15.0; // Example: 15:1 reduction
  public static final double MINIMUM_HEIGHT = 0.0; // Example: 0.0 meters
  public static final double MAXIMUM_HEIGHT = 2.0; // Example: 2.0 meters
  public static final double MASS = 5.0; // Example: 5.0 kg

  public final TalonFX leader = new TalonFX(23); // Changed CAN IDs to avoid conflicts
  public final TalonFX follower = new TalonFX(24);
  public final CANcoder extensionEncoder = new CANcoder(25);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals for all our sensor inputs
  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Angle> encoderPosition = extensionEncoder.getPosition();
  private final StatusSignal<AngularVelocity> encoderVelocity = extensionEncoder.getVelocity();

  // Debouncers for connectivity checking
  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer followerDebounce = new Debouncer(0.5);
  private final Debouncer encoderDebounce = new Debouncer(0.5);

  public ArmExtensionIOCTRE() {
    var config = new TalonFXConfiguration();

    // Configure current limits to protect motors
    config.CurrentLimits.StatorCurrentLimit = 40.0; // Increased for elevator load
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Use brake mode for elevator to prevent backdriving
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Configure feedback from CANCoder
    config.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // PID configuration - Adjusted for elevator control
    config.Slot0.kP = 8.0; // Increased for better position holding
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.5; // Added some derivative control for oscillation damping
    config.Slot0.kG = 0.12; // Added gravity feed forward

    // Apply configurations
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    // Set up follower motor
    follower.setControl(new Follower(leader.getDeviceID(), false));

    // Configure status signal update rates
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50 Hz update rate
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        followerStatorCurrent,
        leaderSupplyCurrent,
        followerSupplyCurrent,
        encoderPosition,
        encoderVelocity);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    // Refresh all sensor data
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    var followerStatus = BaseStatusSignal.refreshAll(followerStatorCurrent, followerSupplyCurrent);
    var encoderStatus = BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    // Update connection status
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());
    inputs.encoderConnected = encoderDebounce.calculate(encoderStatus.isOK());

    // Convert motor rotations to linear distance
    inputs.leaderPosition = leaderPosition.getValue().times(DRUM_CIRCUMFERENCE_METERS / GEAR_RATIO);
    inputs.leaderVelocity = leaderVelocity.getValue().times(DRUM_CIRCUMFERENCE_METERS / GEAR_RATIO);

    // Get encoder readings (assumed to be already in linear units)
    inputs.encoderPosition = encoderPosition.getValue();
    inputs.encoderVelocity = encoderVelocity.getValue();

    // Update electrical measurements
    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    // Optimize CAN bus usage
    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);
  }

  @Override
  public void setVoltage(Voltage volts) {
    leader.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  @Override
  public void setPosition(Distance distance) {
    // Convert linear distance to motor rotations
    double rotations = distance.in(Meters) * GEAR_RATIO / DRUM_CIRCUMFERENCE_METERS;
    leader.setControl(new PositionVoltage(rotations));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
