// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Controller class for the operator, focusing on mechanism controls. */
public class OperatorController extends TunableController {

  /**
   * Creates a new OperatorController with specified type of input response.
   *
   * @param port The port index on the Driver Station
   * @param type The response curve type (LINEAR, QUADRATIC, CUBIC)
   */
  public OperatorController(int port, TunableControllerType type) {
    super(port);
    withControllerType(type);
  }

  /** Gets the Floor Intake button */
  public Trigger floorIntakeButton() {
    return leftBumper();
  }
}
