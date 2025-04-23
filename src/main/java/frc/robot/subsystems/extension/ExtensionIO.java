// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.extension;

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

public interface ExtensionIO {
  @AutoLog
  public static class ExtensionIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public Angle leaderPosition = Rotations.of(0);
    public Angle leaderRotorPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity leaderRotorVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);

    public Distance extensionDistance = Inches.of(0);

    /** Current control slot being used (0 or 1) */
    public int activeControlSlot = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExtensionIOInputs inputs) {}

  /** Run closed loop at the specified distance. */
  public default void setDistance(Distance distance) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void setBrakeMode(boolean enabled) {}

  /**
   * Sets which control slot to use. Slot 0 is typically for no game piece, Slot 1 is for when
   * holding a game piece.
   *
   * @param slot The slot number to use (0 or 1)
   */
  public default void setControlSlot(int slot) {}
}
