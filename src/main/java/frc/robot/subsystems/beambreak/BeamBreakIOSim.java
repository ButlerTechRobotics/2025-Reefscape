// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;

/** Implementation of BeamBreakIO for simulation. */
public class BeamBreakIOSim implements BeamBreakIO {
  private final DigitalInput beamBreakSensor;
  private final DIOSim beamBreakSim;
  private boolean lastBeamState = false;
  private double lastStateChangeTime = 0.0;

  /**
   * Creates a new BeamBreakIOSim.
   *
   * @param dioChannel The DIO channel for the beam break sensor
   */
  public BeamBreakIOSim(int dioChannel) {
    beamBreakSensor = new DigitalInput(dioChannel);
    beamBreakSim = new DIOSim(beamBreakSensor);
    // Start with beam unbroken (no game piece)
    beamBreakSim.setValue(true);
  }

  @Override
  public void updateInputs(BeamBreakIOInputsAutoLogged inputs) {
    // Get the value from simulation
    inputs.rawDioValue = beamBreakSensor.get();

    // For simulation, we're assuming true = beam NOT broken (no game piece)
    // false = beam broken (game piece present)
    inputs.beamBroken = !inputs.rawDioValue;

    // Track state changes
    if (inputs.beamBroken != lastBeamState) {
      lastStateChangeTime = Timer.getFPGATimestamp();
      lastBeamState = inputs.beamBroken;
    }

    inputs.lastStateChangeTimestamp = lastStateChangeTime;
  }

  /**
   * Simulates a game piece being detected by the beam break.
   *
   * @param gamePresent True if a game piece is present, false otherwise
   */
  public void setGamePiece(boolean hasGamePiece) {
    // When game piece is present, beam is broken, so DIO reads false
    beamBreakSim.setValue(hasGamePiece);
  }
}
