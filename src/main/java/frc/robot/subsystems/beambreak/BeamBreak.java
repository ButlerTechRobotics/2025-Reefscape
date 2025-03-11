// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem that manages a beam break sensor to detect game pieces. */
public class BeamBreak extends SubsystemBase {
  private final BeamBreakIO io;
  private final BeamBreakIOInputsAutoLogged inputs = new BeamBreakIOInputsAutoLogged();

  private boolean hasGamePiece = false;
  private boolean prevHasGamePiece = false;
  private double gamePickupTimestamp = 0.0;

  /**
   * Creates a new BeamBreak subsystem.
   *
   * @param io The BeamBreakIO implementation
   */
  public BeamBreak(BeamBreakIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Save previous state
    prevHasGamePiece = hasGamePiece;

    // Update inputs
    io.updateInputs(inputs);

    // Log inputs to AdvantageScope
    Logger.processInputs("BeamBreak", inputs);

    // Update game piece state
    hasGamePiece = inputs.beamBroken;

    // Log when state changes
    if (hasGamePiece != prevHasGamePiece) {
      if (hasGamePiece) {
        gamePickupTimestamp = inputs.lastStateChangeTimestamp;
      }
    }
    // Log game piece state
    Logger.recordOutput("BeamBreak/HasGamePiece", hasGamePiece);
    Logger.recordOutput("BeamBreak/GamePickupTimestamp", gamePickupTimestamp);
  }

  /**
   * Returns whether the system currently detects a game piece.
   *
   * @return True if a game piece is detected, false otherwise
   */
  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  /**
   * Gets the timestamp of when the last game piece was acquired.
   *
   * @return FPGA timestamp of the last game piece acquisition
   */
  public double getGamePickupTimestamp() {
    return gamePickupTimestamp;
  }

  /**
   * Simulates a game piece being detected by the beam break.
   *
   * @param gamePresent True if a game piece is present, false otherwise
   */
  public void setGamePiece(boolean hasGamePiece) {
    // When game piece is present, beam is broken, so DIO reads false
    io.setGamePiece(!hasGamePiece);
  }
}
