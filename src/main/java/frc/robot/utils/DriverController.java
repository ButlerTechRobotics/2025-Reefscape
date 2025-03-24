// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Controller class for the driver, focusing on drive controls. */
public class DriverController extends TunableController {

  /**
   * Creates a new DriverController with specified type of input response.
   *
   * @param port The port index on the Driver Station
   * @param type The response curve type (LINEAR, QUADRATIC, CUBIC)
   */
  public DriverController(int port, TunableControllerType type) {
    super(port);
    withControllerType(type);
  }

  /** Gets trigger for intake. */
  public Trigger intake() {
    return leftBumper();
  }

  /** Gets trigger for shoot */
  public Trigger shoot() {
    return leftTrigger();
  }

  /** Gets trigger for reef alignment left. */
  public Trigger reefAlignLeft() {
    return leftStick();
  }

  /** Gets trigger for reef alignment right. */
  public Trigger reefAlignRight() {
    return rightStick();
  }

  /** Gets trigger for reset heading. */
  public Trigger resetHeading() {
    return start();
  }
}
