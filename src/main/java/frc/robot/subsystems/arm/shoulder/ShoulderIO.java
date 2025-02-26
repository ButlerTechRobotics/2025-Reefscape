// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.shoulder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  @AutoLog
  public static class ShoulderIOInputs {
    // Connection status for all motors
    public boolean brLeaderConnected = false;
    public boolean blFollowerConnected = false;
    public boolean frFollowerConnected = false;
    public boolean flFollowerConnected = false;
    public boolean encoderConnected = false;

    // Position measurements
    public Angle brLeaderPosition = Rotations.of(0);
    public Angle brLeaderRotorPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    // Velocity measurements
    public AngularVelocity brLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity brLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    // Voltage and current measurements
    public Voltage appliedVoltage = Volts.of(0.0);

    // Stator currents for all motors
    public Current brLeaderStatorCurrent = Amps.of(0);
    public Current blFollowerStatorCurrent = Amps.of(0);
    public Current frFollowerStatorCurrent = Amps.of(0);
    public Current flFollowerStatorCurrent = Amps.of(0);

    // Supply currents for all motors
    public Current brLeaderSupplyCurrent = Amps.of(0);
    public Current blFollowerSupplyCurrent = Amps.of(0);
    public Current frFollowerSupplyCurrent = Amps.of(0);
    public Current flFollowerSupplyCurrent = Amps.of(0);

    // Derived angle measurement
    public Angle shoulderAngle = Rotations.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShoulderIOInputs inputs) {}

  /** Run closed loop at the specified position. */
  public default void setPosition(Angle angle) {}

  /** Stop in open loop. */
  public default void stop() {}
}
