// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    // Shoulder motor status
    public boolean shoulderLeaderConnected = false;
    public boolean shoulderFollowerConnected = false;
    public boolean shoulderEncoderConnected = false;

    // Wrist motor status
    public boolean wristLeaderConnected = false;
    public boolean wristEncoderConnected = false;

    // Extension motor status
    public boolean extensionLeaderConnected = false;
    public boolean extensionFollowerConnected = false;
    public boolean extensionEncoderConnected = false;

    // Shoulder positions
    public Angle shoulderLeaderPosition = Rotations.of(0);
    public Angle shoulderLeaderRotorPosition = Rotations.of(0);
    public Angle shoulderEncoderPosition = Rotations.of(0);

    // Wrist positions
    public Angle wristLeaderPosition = Rotations.of(0);
    public Angle wristLeaderRotorPosition = Rotations.of(0);
    public Angle wristEncoderPosition = Rotations.of(0);

    // Extension positions
    public Angle extensionLeaderPosition = Rotations.of(0);
    public Angle extensionLeaderRotorPosition = Rotations.of(0);
    public Angle extensionEncoderPosition = Rotations.of(0);

    // Shoulder velocities
    public AngularVelocity shoulderLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity shoulderLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity shoulderEncoderVelocity = RotationsPerSecond.of(0);

    // Wrist velocities
    public AngularVelocity wristLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity wristLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity wristEncoderVelocity = RotationsPerSecond.of(0);

    // Extension velocities
    public AngularVelocity extensionLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity extensionLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity extensionEncoderVelocity = RotationsPerSecond.of(0);

    // Shoulder electrical measurements
    public Voltage shoulderAppliedVoltage = Volts.of(0.0);
    public Current shoulderLeaderStatorCurrent = Amps.of(0);
    public Current shoulderFollowerStatorCurrent = Amps.of(0);
    public Current shoulderLeaderSupplyCurrent = Amps.of(0);
    public Current shoulderFollowerSupplyCurrent = Amps.of(0);

    // Wrist electrical measurements
    public Voltage wristAppliedVoltage = Volts.of(0.0);
    public Current wristLeaderStatorCurrent = Amps.of(0);
    public Current wristFollowerStatorCurrent = Amps.of(0);
    public Current wristLeaderSupplyCurrent = Amps.of(0);
    public Current wristFollowerSupplyCurrent = Amps.of(0);

    // Extension electrical measurements
    public Voltage extensionAppliedVoltage = Volts.of(0.0);
    public Current extensionLeaderStatorCurrent = Amps.of(0);
    public Current extensionFollowerStatorCurrent = Amps.of(0);
    public Current extensionLeaderSupplyCurrent = Amps.of(0);
    public Current extensionFollowerSupplyCurrent = Amps.of(0);

    // Processed measurements for subsystem use
    public Angle shoulderAngle = Rotations.of(0);
    public Angle wristAngle = Rotations.of(0);
    public Distance extensionDistance = Inches.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Run closed loop to the specified shoulder angle, wrist angle, and extension distance.
   *
   * @param shoulderAngle The target shoulder angle
   * @param wristAngle The target wrist angle
   * @param extensionDistance The target extension distance
   */
  public default void setPosition(
      Angle shoulderAngle, Angle wristAngle, Distance extensionDistance) {}

  /** Stop all motors in open loop. */
  public default void stop() {}
}
