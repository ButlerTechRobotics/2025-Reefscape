// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
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

  @AutoLogOutput private boolean brakeModeEnabled = true;

  // Arm is vertical when the shoulder is between 45 and 90 degrees
  private boolean isVertical = false;

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

    // Update which slot is being used based on game piece status
    io.setControlSlot(isVertical ? 0 : 1);

    // Log which control slot is being used
    Logger.recordOutput("Extension/UsingVerticalSlot", isVertical);
  }

  /**
   * Sets whether the arm currently is vertical. This will switch between PID slots for different
   * control characteristics.
   *
   * @param isVertical true if arm is vertical, false otherwise
   */
  public void setIsVertical(boolean isVertical) {
    this.isVertical = isVertical;
  }

  /**
   * Gets whether the extension currently is vertical.
   *
   * @return true if arm is vertical, false otherwise
   */
  @AutoLogOutput(key = "Extension/IsVertical")
  public boolean getIsVertical() {
    return isVertical;
  }

  /**
   * Runs the arm in closed-loop distance mode to the specified angle.
   *
   * @param distance The target angle distance
   */
  private void setDistance(Distance distance) {
    io.setDistance(distance);
  }

  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
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

  /**
   * Sets the extension to a specific length.
   *
   * @param length The desired extension length
   */
  public void setLength(Distance length) {
    io.setDistance(length);
  }

  /**
   * Creates a command to set the extension to a specific length.
   *
   * @param length The desired extension length
   * @return Command to set the length
   */
  public Command setLengthCommand(Distance length) {
    return Commands.runOnce(() -> setLength(length))
        .withName("SetExtensionLength(" + length.in(Inches) + ")");
  }

  /** Enumeration of available extension distances with their corresponding target distances. */
  public enum ExtensionPosition {
    // Common positions
    STOP(Inches.of(0)),
    STOW(Inches.of(0)),
    STANDBY(Inches.of(0)),
    CLIMB(Inches.of(0)),
    CLIMB_DOWN(Inches.of(0)),

    // Coral positions
    CORAL_FLOOR_INTAKE(Inches.of(14)),
    CORAL_STATION_INTAKE(Inches.of(30)),
    CORAL_L1(Inches.of(0)),
    CORAL_L1BACK(Inches.of(0)),
    CORAL_L2(Inches.of(0)),
    CORAL_L2BACK(Inches.of(0)),
    CORAL_L3(Inches.of(0)),
    CORAL_L3BACK(Inches.of(6)),
    CORAL_L4BACK(Inches.of(60)),

    // Algae positions
    ALGAE_FLOOR_INTAKE(Inches.of(14)),
    ALGAE_SCORE(Inches.of(0)),
    ALGAE_L1(Inches.of(0)),
    ALGAE_L2(Inches.of(12));

    private final Distance targetDistance;
    private final Distance lengthTolerance;

    ExtensionPosition(Distance targetDistance, Distance lengthTolerance) {
      this.targetDistance = targetDistance;
      this.lengthTolerance = lengthTolerance;
    }

    ExtensionPosition(Distance targetDistance) {
      this(targetDistance, Inches.of(4)); // 2 degree default tolerance
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

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  // Command that runs the appropriate routine based on the current distance
  private final Command currentCommand =
      new SelectCommand<>(
          Map.ofEntries(
              // Common positions
              Map.entry(
                  ExtensionPosition.STOP, Commands.runOnce(this::stop).withName("Stop Extension")),
              Map.entry(ExtensionPosition.STOW, createPositionCommand(ExtensionPosition.STOW)),
              Map.entry(
                  ExtensionPosition.STANDBY, createPositionCommand(ExtensionPosition.STANDBY)),
              Map.entry(ExtensionPosition.CLIMB, createPositionCommand(ExtensionPosition.CLIMB)),
              Map.entry(
                  ExtensionPosition.CLIMB_DOWN,
                  createPositionCommand(ExtensionPosition.CLIMB_DOWN)),

              // Coral positions
              Map.entry(
                  ExtensionPosition.CORAL_FLOOR_INTAKE,
                  createPositionCommand(ExtensionPosition.CORAL_FLOOR_INTAKE)),
              Map.entry(
                  ExtensionPosition.CORAL_STATION_INTAKE,
                  createPositionCommand(ExtensionPosition.CORAL_STATION_INTAKE)),
              Map.entry(
                  ExtensionPosition.CORAL_L1, createPositionCommand(ExtensionPosition.CORAL_L1)),
              Map.entry(
                  ExtensionPosition.CORAL_L1BACK,
                  createPositionCommand(ExtensionPosition.CORAL_L1BACK)),
              Map.entry(
                  ExtensionPosition.CORAL_L2, createPositionCommand(ExtensionPosition.CORAL_L2)),
              Map.entry(
                  ExtensionPosition.CORAL_L2BACK,
                  createPositionCommand(ExtensionPosition.CORAL_L2BACK)),
              Map.entry(
                  ExtensionPosition.CORAL_L3, createPositionCommand(ExtensionPosition.CORAL_L3)),
              Map.entry(
                  ExtensionPosition.CORAL_L3BACK,
                  createPositionCommand(ExtensionPosition.CORAL_L3BACK)),
              Map.entry(
                  ExtensionPosition.CORAL_L4BACK,
                  createPositionCommand(ExtensionPosition.CORAL_L4BACK)),

              // Algae positions
              Map.entry(
                  ExtensionPosition.ALGAE_FLOOR_INTAKE,
                  createPositionCommand(ExtensionPosition.ALGAE_FLOOR_INTAKE)),
              Map.entry(
                  ExtensionPosition.ALGAE_SCORE,
                  createPositionCommand(ExtensionPosition.ALGAE_SCORE)),
              Map.entry(
                  ExtensionPosition.ALGAE_L1, createPositionCommand(ExtensionPosition.ALGAE_L1)),
              Map.entry(
                  ExtensionPosition.ALGAE_L2, createPositionCommand(ExtensionPosition.ALGAE_L2))),
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
    return getPosition().isNear(currentMode.targetDistance, currentMode.lengthTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  public Distance targetDistance() {
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
   * @return Command to move the extension to standby position
   */
  public final Command standby() {
    return setPositionCommand(ExtensionPosition.STANDBY);
  }

  /**
   * @return Command to move the extension to coral floor intake position
   */
  public final Command coralFloorIntake() {
    return setPositionCommand(ExtensionPosition.CORAL_FLOOR_INTAKE);
  }

  /**
   * @return Command to move the extension to coral station intake position
   */
  public final Command coralStationIntake() {
    return setPositionCommand(ExtensionPosition.CORAL_STATION_INTAKE);
  }

  /**
   * @return Command to move the extension to CORAL_L1 position
   */
  public final Command coralL1() {
    return setPositionCommand(ExtensionPosition.CORAL_L1);
  }

  /**
   * @return Command to move the extension to CORAL_L1BACK position
   */
  public final Command coralL1Back() {
    return setPositionCommand(ExtensionPosition.CORAL_L1BACK);
  }

  /**
   * @return Command to move the extension to CORAL_L2 position
   */
  public final Command coralL2() {
    return setPositionCommand(ExtensionPosition.CORAL_L2);
  }

  /**
   * @return Command to move the extension to CORAL_L2BACK position
   */
  public final Command coralL2Back() {
    return setPositionCommand(ExtensionPosition.CORAL_L2BACK);
  }

  /**
   * @return Command to move the extension to CORAL_L3 position
   */
  public final Command coralL3() {
    return setPositionCommand(ExtensionPosition.CORAL_L3);
  }

  /**
   * @return Command to move the extension to CORAL_L3BACK position
   */
  public final Command coralL3Back() {
    return setPositionCommand(ExtensionPosition.CORAL_L3BACK);
  }

  /**
   * @return Command to move the extension to CORAL_L4BACK position
   */
  public final Command coralL4Back() {
    return setPositionCommand(ExtensionPosition.CORAL_L4BACK);
  }

  /**
   * @return Command to move the extension to ALGAE_FLOOR_INTAKE position
   */
  public final Command algaeFloorIntake() {
    return setPositionCommand(ExtensionPosition.ALGAE_FLOOR_INTAKE);
  }

  /**
   * @return Command to move the extension to ALGAE_SCORE position
   */
  public final Command algaeScore() {
    return setPositionCommand(ExtensionPosition.ALGAE_SCORE);
  }

  /**
   * @return Command to move the extension to ALGAE_L1 position
   */
  public final Command algaeL1() {
    return setPositionCommand(ExtensionPosition.ALGAE_L1);
  }

  /**
   * @return Command to move the extension to ALGAE_L2 position
   */
  public final Command algaeL2() {
    return setPositionCommand(ExtensionPosition.ALGAE_L2);
  }

  /**
   * @return Command to move the extension to climb position
   */
  public final Command climb() {
    return setPositionCommand(ExtensionPosition.CLIMB);
  }

  /**
   * @return Command to move the extension to climb down position
   */
  public final Command climbDown() {
    return setPositionCommand(ExtensionPosition.CLIMB_DOWN);
  }

  /**
   * Checks if the shoulder is in a vertical position (greater than 45 degrees).
   *
   * @return true if shoulder is greater than 45 degrees, false otherwise
   */
  @AutoLogOutput(key = "Extension/IsExtended")
  public boolean isExtended() {
    // Convert current position to degrees (0.125 rotations = 45°)
    double currentInches = getPosition().in(Inches);
    return currentInches >= 22;
  }
}
