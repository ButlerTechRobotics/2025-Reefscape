// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

/** Interface for beam break sensors to track game piece presence. */
public interface BeamBreakIO {
  /** Data structure for beam break inputs. */
  @AutoLog
  public static class BeamBreakIOInputs {
    /** Whether the beam is broken (true if game piece detected) */
    public boolean beamBroken = false;
    /** The raw DIO value from the beam break sensor */
    public boolean rawDioValue = false;
    /** Timestamp of the last time the beam state changed */
    public double lastStateChangeTimestamp = 0.0;
  }

  public default void setGamePiece(boolean hasGamePiece) {}

  /**
   * Updates the set of inputs.
   *
   * @param inputs The input object to be updated
   */
  public default void updateInputs(BeamBreakIOInputsAutoLogged inputs) {}
}
