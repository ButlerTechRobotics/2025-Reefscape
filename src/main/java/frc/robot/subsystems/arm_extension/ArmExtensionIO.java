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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the arm extension (elevator) hardware abstraction. This interface defines the
 * contract between the subsystem and its hardware implementations, allowing for different
 * implementations (real, simulated, etc.) while maintaining consistent behavior.
 */
public interface ArmExtensionIO {
  @AutoLog
  public static class ArmExtensionIOInputs {
    // Connection status flags
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public boolean encoderConnected = false;

    // Position measurements
    public Angle leaderPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    // Velocity measurements
    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    // Electrical measurements
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);
  }

  /** Updates the set of loggable inputs from the hardware. */
  public default void updateInputs(ArmExtensionIOInputs inputs) {}

  /**
   * Runs the extension mechanism in open loop at the specified voltage.
   *
   * @param volts The voltage to apply to the motors
   */
  public default void setVoltage(Voltage volts) {}

  /**
   * Runs the extension mechanism in closed loop to reach the specified position.
   *
   * @param distance The target distance to extend to
   */
  public default void setPosition(Distance distance) {}

  /** Stops the extension mechanism motors in open loop. */
  public default void stop() {}
}
