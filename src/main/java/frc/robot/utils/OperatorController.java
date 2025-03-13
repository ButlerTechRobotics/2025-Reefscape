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

  /** Gets the Coral L1 button */
  public Trigger coralL1() {
    return a();
  }

  /** Gets the Coral L2 button */
  public Trigger coralL2() {
    return x();
  }

  /** Gets the Coral L3 button */
  public Trigger coralL3() {
    return b();
  }

  /** Gets the Coral L4 button */
  public Trigger coralL4() {
    return y();
  }

  /** Gets the Algae L1 button */
  public Trigger algaeL1() {
    return povDown();
  }

  /** Gets the Algae L2 button */
  public Trigger algaeL2() {
    return povUp();
  }

  /** Gets the Algae Intake button */
  public Trigger algaeFloorIntake() {
    return povLeft();
  }

  /** Gets the Algae Score button */
  public Trigger algaeScore() {
    return povRight();
  }
}
