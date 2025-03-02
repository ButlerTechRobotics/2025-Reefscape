// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/** Implementation of BeamBreakIO for real robot hardware. */
public class BeamBreakIOReal implements BeamBreakIO {
  private final DigitalInput beamBreakSensor;
  private boolean lastBeamState = false;
  private double lastStateChangeTime = 0.0;

  /**
   * Creates a new BeamBreakIOReal.
   *
   * @param dioChannel The DIO channel for the beam break sensor
   */
  public BeamBreakIOReal(int dioChannel) {
    beamBreakSensor = new DigitalInput(dioChannel);
  }

  @Override
  public void updateInputs(BeamBreakIOInputsAutoLogged inputs) {
    // Get the current raw value (may need to be inverted depending on your sensor)
    inputs.rawDioValue = beamBreakSensor.get();

    // Depending on your sensor, you might need to invert the value
    // For this example, we're assuming true = beam broken (game piece present)
    inputs.beamBroken = !inputs.rawDioValue; // Invert if your sensor reads opposite

    // Track state changes
    if (inputs.beamBroken != lastBeamState) {
      lastStateChangeTime = Timer.getFPGATimestamp();
      lastBeamState = inputs.beamBroken;
    }

    inputs.lastStateChangeTimestamp = lastStateChangeTime;
  }
}
