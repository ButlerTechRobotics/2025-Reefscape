// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
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
 * The Extension subsystem controls a dual-motor arm mechanism for game piece manipulation. It
 * supports multiple distances for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Extension extends SubsystemBase {
  // Hardware interface and inputs
  private final ExtensionIO io;
  private final ExtensionIOInputsAutoLogged inputs;

  // Current arm distance mode
  private ExtensionPosition currentMode = ExtensionPosition.STOW;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Extension leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Extension follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Extension encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Extension subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Extension(ExtensionIO io) {
    this.io = io;
    this.inputs = new ExtensionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Extension", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the arm in closed-loop distance mode to the specified angle.
   *
   * @param distance The target angle distance
   */
  private void setDistance(Distance distance) {
    io.setDistance(distance);
  }

  /** Stops the arm motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current distance of the arm.
   *
   * @return The current angular distance
   */
  @AutoLogOutput
  public Distance getPosition() {
    return inputs.extensionDistance;
  }

  /** Enumeration of available arm distances with their corresponding target angles. */
  public enum ExtensionPosition {
    STOP(Inches.of(0)), // Stop the arm
    STOW(Inches.of(0)), // Stow the arm
    FLOOR_INTAKE(Inches.of(0)), // Position for intaking from floor
    SOURCE_INTAKE(Inches.of(0)), // Position for intaking from source
    L1(Inches.of(12)), // Position for scoring in L1
    L1Back(Inches.of(12)), // Position for scoring in L1Back
    L2(Inches.of(24)), // Position for scoring in L2
    L2Back(Inches.of(24)), // Position for scoring in L2Back
    L3(Inches.of(36)), // Position for scoring in L3
    L3Back(Inches.of(36)), // Position for scoring in L3Back
    L4(Inches.of(48)), // Position for scoring in L4
    L4Back(Inches.of(48)), // Position for scoring in L4Back
    CLIMB(Inches.of(0)); // Position for climbing

    private final Distance targetDistance;
    private final Distance angleTolerance;

    ExtensionPosition(Distance targetDistance, Distance angleTolerance) {
      this.targetDistance = targetDistance;
      this.angleTolerance = angleTolerance;
    }

    ExtensionPosition(Distance targetDistance) {
      this(targetDistance, Inches.of(2)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current arm distance mode.
   *
   * @return The current ExtensionPosition
   */
  public ExtensionPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new arm distance and schedules the corresponding command.
   *
   * @param distance The desired ExtensionPosition
   */
  public void setExtensionPosition(ExtensionPosition distance) {
    if (currentMode != distance) {
      if (currentCommand != null) {
        currentCommand.cancel();
      }
      currentMode = distance;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current distance
  private final Command currentCommand =
      new SelectCommand<>(
          Map.ofEntries(
              Map.entry(
                  ExtensionPosition.STOP, Commands.runOnce(this::stop).withName("Stop Extension")),
              Map.entry(ExtensionPosition.STOW, createPositionCommand(ExtensionPosition.STOW)),
              Map.entry(
                  ExtensionPosition.FLOOR_INTAKE,
                  createPositionCommand(ExtensionPosition.FLOOR_INTAKE)),
              Map.entry(
                  ExtensionPosition.SOURCE_INTAKE,
                  createPositionCommand(ExtensionPosition.SOURCE_INTAKE)),
              Map.entry(ExtensionPosition.L1, createPositionCommand(ExtensionPosition.L1)),
              Map.entry(ExtensionPosition.L1Back, createPositionCommand(ExtensionPosition.L1Back)),
              Map.entry(ExtensionPosition.L2, createPositionCommand(ExtensionPosition.L2)),
              Map.entry(ExtensionPosition.L2Back, createPositionCommand(ExtensionPosition.L2Back)),
              Map.entry(ExtensionPosition.L3, createPositionCommand(ExtensionPosition.L3)),
              Map.entry(ExtensionPosition.L3Back, createPositionCommand(ExtensionPosition.L3Back)),
              Map.entry(ExtensionPosition.L4, createPositionCommand(ExtensionPosition.L4)),
              Map.entry(ExtensionPosition.L4Back, createPositionCommand(ExtensionPosition.L4Back)),
              Map.entry(ExtensionPosition.CLIMB, createPositionCommand(ExtensionPosition.CLIMB))),
          this::getMode);

  /**
   * Creates a command for a specific arm distance that moves the arm and checks the target
   * distance.
   *
   * @param distance The arm distance to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ExtensionPosition distance) {
    return Commands.runOnce(() -> setDistance(distance.targetDistance))
        .withName("Move to " + distance.toString());
  }

  /**
   * Checks if the arm is at its target distance.
   *
   * @return true if at target distance, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ExtensionPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetDistance, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Distance targetDistance() {
    return currentMode.targetDistance;
  }

  /**
   * Creates a command to set the arm to a specific distance.
   *
   * @param distance The desired arm distance
   * @return Command to set the distance
   */
  private Command setPositionCommand(ExtensionPosition distance) {
    return Commands.runOnce(() -> setExtensionPosition(distance))
        .withName("SetExtensionPosition(" + distance.toString() + ")");
  }

  /** Factory methods for common distance commands */

  /**
   * @return Command to stop the extension
   */
  public final Command stopCommand() {
    return setPositionCommand(ExtensionPosition.STOP);
  }

  /**
   * @return Command to move the extension to stow position
   */
  public final Command stow() {
    return setPositionCommand(ExtensionPosition.STOW);
  }

  /**
   * @return Command to move the extension to floor intake position
   */
  public final Command intake() {
    return setPositionCommand(ExtensionPosition.FLOOR_INTAKE);
  }

  /**
   * @return Command to move the extension to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(ExtensionPosition.L1);
  }

  /**
   * @return Command to move the extension to L1Back scoring position
   */
  public final Command L1Back() {
    return setPositionCommand(ExtensionPosition.L1Back);
  }

  /**
   * @return Command to move the extension to L2 scoring position
   */
  public final Command L2() {
    return setPositionCommand(ExtensionPosition.L2);
  }

  /**
   * @return Command to move the extension to L2Back scoring position
   */
  public final Command L2Back() {
    return setPositionCommand(ExtensionPosition.L2Back);
  }

  /**
   * @return Command to move the extension to L3 scoring position
   */
  public final Command L3() {
    return setPositionCommand(ExtensionPosition.L3);
  }

  /**
   * @return Command to move the extension to L3Back scoring position
   */
  public final Command L3Back() {
    return setPositionCommand(ExtensionPosition.L3Back);
  }

  /**
   * @return Command to move the extension to L4 scoring position
   */
  public final Command L4() {
    return setPositionCommand(ExtensionPosition.L4);
  }

  /**
   * @return Command to move the extension to L4Back scoring position
   */
  public final Command L4Back() {
    return setPositionCommand(ExtensionPosition.L4Back);
  }

  /**
   * @return Command to move the extension to climb position
   */
  public final Command climb() {
    return setPositionCommand(ExtensionPosition.CLIMB);
  }
}
