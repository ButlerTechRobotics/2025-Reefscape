// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.claw_joint;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The ClawJoint subsystem controls a dual-motor claw joint mechanism for game piece manipulation.
 * It supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class ClawJoint extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawJointIO io;
  private final ClawJointIOInputsAutoLogged inputs;

  // Current claw joint position mode
  private ClawJointPosition currentMode = ClawJointPosition.INTAKE;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Claw-Joint leader motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Claw-Joint encoder isn't connected", AlertType.kError);

  /**
   * Creates a new ClawJoint subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the claw joint
   */
  public ClawJoint(ClawJointIO io) {
    this.io = io;
    this.inputs = new ClawJointIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw-Joint", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the claw joint in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  private void setPosition(Angle position) {
    io.setPosition(position);
  }

  /** Stops the claw joint motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current position of the claw joint.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /** Enumeration of available claw joint positions with their corresponding target angles. */
  private enum ClawJointPosition {
    STOP(Degrees.of(0)), // Stop the claw joint
    INTAKE(Degrees.of(0)), // Claw joint tucked in
    L1(Degrees.of(90)), // Position for scoring in L1
    L2(Degrees.of(135)), // Position for scoring in L2
    L3(Degrees.of(135)), // Position for scoring in L3
    L4(Degrees.of(180)); // Position for scoring in L4

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ClawJointPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ClawJointPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current claw joint position mode.
   *
   * @return The current ClawJointPosition
   */
  public ClawJointPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new claw joint position and schedules the corresponding command.
   *
   * @param position The desired ClawJointPosition
   */
  private void setClawJointPosition(ClawJointPosition position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ClawJointPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop ClawJoint"),
              ClawJointPosition.INTAKE,
              createPositionCommand(ClawJointPosition.INTAKE),
              ClawJointPosition.L1,
              createPositionCommand(ClawJointPosition.L1),
              ClawJointPosition.L2,
              createPositionCommand(ClawJointPosition.L2),
              ClawJointPosition.L3,
              createPositionCommand(ClawJointPosition.L3),
              ClawJointPosition.L4,
              createPositionCommand(ClawJointPosition.L4)),
          this::getMode);

  /**
   * Creates a command for a specific claw joint position that moves the claw joint and checks the
   * target position.
   *
   * @param position The claw joint position to create a command for
   * @return A command that implements the claw joint movement
   */
  private Command createPositionCommand(ClawJointPosition position) {
    return Commands.runOnce(() -> setPosition(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the claw joint is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ClawJointPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Angle targetAngle() {
    return currentMode.targetAngle;
  }

  /**
   * Creates a command to set the claw joint to a specific position.
   *
   * @param position The desired claw joint position
   * @return Command to set the position
   */
  private Command setPositionCommand(ClawJointPosition position) {
    return Commands.runOnce(() -> setClawJointPosition(position))
        .withName("SetClawJointPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to move the claw joint to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(ClawJointPosition.L1);
  }

  /**
   * @return Command to move the claw joint to L2 scoring position
   */
  public final Command L2() {
    return setPositionCommand(ClawJointPosition.L2);
  }

  /**
   * @return Command to move the claw joint to L3 position
   */
  public final Command L3() {
    return setPositionCommand(ClawJointPosition.L3);
  }

  /**
   * @return Command to move the claw joint to L4 position
   */
  public final Command L4() {
    return setPositionCommand(ClawJointPosition.L4);
  }

  /**
   * @return Command to intake the claw joint
   */
  public final Command intake() {
    return setPositionCommand(ClawJointPosition.INTAKE);
  }

  /**
   * @return Command to stop the claw joint
   */
  public final Command stopCommand() {
    return setPositionCommand(ClawJointPosition.STOP);
  }
}
