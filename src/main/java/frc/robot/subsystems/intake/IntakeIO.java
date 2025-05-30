// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

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

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leaderConnected = false;
    public boolean frontCANrangeConnected = false;
    public boolean backCANrangeConnected = false;

    public Angle leaderPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);

    public boolean hasGamePiece = false;
    public boolean hasFrontGamePiece = false;
    public boolean hasBackGamePiece = false;

    public Distance frontCANrangeDistance = Inches.of(0);
    public Distance backCANrangeDistance = Inches.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  default void setCANrangeDistanceSim(Distance distance) {
    // Do nothing by default
  }
}
