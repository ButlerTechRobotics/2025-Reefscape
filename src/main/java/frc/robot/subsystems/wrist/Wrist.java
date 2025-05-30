// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Wrist subsystem controls a dual-motor wrist mechanism for game piece manipulation. It
 * supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Wrist extends SubsystemBase {
  // Hardware interface and inputs
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;

  // Current wrist position mode
  private WristPosition currentMode = WristPosition.STOW;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Wrist leader motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Wrist encoder isn't connected", AlertType.kError);

  private boolean zeroed = false;

  // Game piece detection flag
  private boolean hasGamePiece = false;

  /**
   * Creates a new Wrist subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the wrist
   */
  public Wrist(WristIO io) {
    this.io = io;
    this.inputs = new WristIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    encoderAlert.set(!inputs.encoderConnected);

    // Update which slot is being used based on game piece status
    io.setControlSlot(hasGamePiece ? 1 : 0);

    // Log which control slot is being used
    Logger.recordOutput("Wrist/UsingGamePieceSlot", hasGamePiece);
  }

  /**
   * Sets whether the robot currently has a game piece. This will switch between PID slots for
   * different control characteristics.
   *
   * @param hasGamePiece true if robot has a game piece, false otherwise
   */
  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
  }

  /**
   * Gets whether the robot currently has a game piece.
   *
   * @return true if robot has a game piece, false otherwise
   */
  @AutoLogOutput(key = "Wrist/HasGamePiece")
  public boolean getHasGamePiece() {
    return hasGamePiece;
  }

  /**
   * Runs the wrist in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  public void setPosition(Angle position) {
    io.setPosition(position);
  }

  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public void setZero() {
    // Set the current position as zero
    io.setEncoderPosition(Rotations.of(0));

    // Mark the wrist as zeroed
    setZeroed(true);

    Logger.recordOutput("Wrist/SetZero", true);
  }

  /** Stops the wrist motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current position of the wrist.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /**
   * Sets the wrist to a specific angle.
   *
   * @param angle The desired angle in Rotations
   */
  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  /**
   * Creates a command to set the wrist to a specific angle.
   *
   * @param angle The desired angle in Rotations
   * @return Command to set the angle
   */
  public Command setAngleCommand(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle))
        .withName("SetWristAngle(" + angle.in(Rotations) + ")");
  }

  /** Enumeration of available wrist positions with their corresponding target angles. */
  public enum WristPosition {
    // Common positions
    STOP(Rotations.of(0)),
    STOW(Rotations.of(0.2)),
    STANDBY(Rotations.of(0.2)),
    CLIMB(Rotations.of(1.65)),
    CLIMB_DOWN(Rotations.of(0.9)),

    // Coral positions
    CORAL_PRE_INTAKE(Rotations.of(1.6)),
    CORAL_FLOOR_INTAKE(Rotations.of(1.6)),
    CORAL_STATION_INTAKE(Rotations.of(1.71)),
    CORAL_L1(Rotations.of(1.45)),
    CORAL_L1BACK(Rotations.of(0)),
    CORAL_L2(Rotations.of(0.8)),
    CORAL_L2BACK(Rotations.of(0.45)),
    CORAL_L3(Rotations.of(0.8)),
    // CORAL_L3BACK(Rotations.of(0.897)),
    CORAL_L3BACK(Rotations.of(0.75)),
    CORAL_L4(Rotations.of(0)),
    CORAL_L4BACK(Rotations.of(0.55)),

    // Algae positions (all 0)
    ALGAE_FLOOR_INTAKE(Rotations.of(1.6)),
    ALGAE_SCORE(Rotations.of(1.6)),
    ALGAE_L1(Rotations.of(1.65)),
    ALGAE_L2(Rotations.of(0.87));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    WristPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    WristPosition(Angle targetAngle) {
      this(targetAngle, Rotations.of(0.25)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current wrist position mode.
   *
   * @return The current WristPosition
   */
  public WristPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new wrist position and schedules the corresponding command.
   *
   * @param position The desired WristPosition
   */
  public void setWristPosition(WristPosition position) {
    if (currentMode != position) {
      if (currentCommand != null) {
        currentCommand.cancel();
      }
      currentMode = position;
      currentCommand.schedule();
    }
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.ofEntries(
              // Common positions
              Map.entry(WristPosition.STOP, Commands.runOnce(this::stop).withName("Stop Wrist")),
              Map.entry(WristPosition.STOW, createPositionCommand(WristPosition.STOW)),
              Map.entry(WristPosition.STANDBY, createPositionCommand(WristPosition.STANDBY)),
              Map.entry(WristPosition.CLIMB, createPositionCommand(WristPosition.CLIMB)),
              Map.entry(WristPosition.CLIMB_DOWN, createPositionCommand(WristPosition.CLIMB_DOWN)),

              // Coral positions
              Map.entry(
                  WristPosition.CORAL_PRE_INTAKE,
                  createPositionCommand(WristPosition.CORAL_PRE_INTAKE)),
              Map.entry(
                  WristPosition.CORAL_FLOOR_INTAKE,
                  createPositionCommand(WristPosition.CORAL_FLOOR_INTAKE)),
              Map.entry(
                  WristPosition.CORAL_STATION_INTAKE,
                  createPositionCommand(WristPosition.CORAL_STATION_INTAKE)),
              Map.entry(WristPosition.CORAL_L1, createPositionCommand(WristPosition.CORAL_L1)),
              Map.entry(
                  WristPosition.CORAL_L1BACK, createPositionCommand(WristPosition.CORAL_L1BACK)),
              Map.entry(WristPosition.CORAL_L2, createPositionCommand(WristPosition.CORAL_L2)),
              Map.entry(
                  WristPosition.CORAL_L2BACK, createPositionCommand(WristPosition.CORAL_L2BACK)),
              Map.entry(WristPosition.CORAL_L3, createPositionCommand(WristPosition.CORAL_L3)),
              Map.entry(
                  WristPosition.CORAL_L3BACK, createPositionCommand(WristPosition.CORAL_L3BACK)),
              Map.entry(
                  WristPosition.CORAL_L4BACK, createPositionCommand(WristPosition.CORAL_L4BACK)),

              // Algae positions
              Map.entry(
                  WristPosition.ALGAE_FLOOR_INTAKE,
                  createPositionCommand(WristPosition.ALGAE_FLOOR_INTAKE)),
              Map.entry(
                  WristPosition.ALGAE_SCORE, createPositionCommand(WristPosition.ALGAE_SCORE)),
              Map.entry(WristPosition.ALGAE_L1, createPositionCommand(WristPosition.ALGAE_L1)),
              Map.entry(WristPosition.ALGAE_L2, createPositionCommand(WristPosition.ALGAE_L2))),
          this::getMode);

  /**
   * Creates a command for a specific wrist position that moves the wrist and checks the target
   * position.
   *
   * @param position The wrist position to create a command for
   * @return A command that implements the wrist movement
   */
  private Command createPositionCommand(WristPosition position) {
    return Commands.runOnce(() -> setPosition(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the wrist is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == WristPosition.STOW) return true;
    return getPosition().isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  public Angle targetAngle() {
    return currentMode.targetAngle;
  }

  /**
   * Creates a command to set the wrist to a specific position.
   *
   * @param position The desired wrist position
   * @return Command to set the position
   */
  private Command setPositionCommand(WristPosition position) {
    return Commands.runOnce(() -> setWristPosition(position))
        .withName("SetWristPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to stop the wrist
   */
  public final Command stopCommand() {
    return setPositionCommand(WristPosition.STOP);
  }

  /**
   * @return Command to move the wrist to stow position
   */
  public final Command stow() {
    return setPositionCommand(WristPosition.STOW);
  }

  /**
   * @return Command to move the wrist to standby position
   */
  public final Command standby() {
    return setPositionCommand(WristPosition.STANDBY);
  }

  /**
   * @return Command to move the wrist to coral pre-intake position
   */
  public final Command coralPreIntake() {
    return setPositionCommand(WristPosition.CORAL_PRE_INTAKE);
  }

  /**
   * @return Command to move the wrist to coral floor intake position
   */
  public final Command coralFloorIntake() {
    return setPositionCommand(WristPosition.CORAL_FLOOR_INTAKE);
  }

  /**
   * @return Command to move the wrist to coral station intake position
   */
  public final Command coralStationIntake() {
    return setPositionCommand(WristPosition.CORAL_STATION_INTAKE);
  }

  /**
   * @return Command to move the wrist to coral L1 scoring position
   */
  public final Command coralL1() {
    return setPositionCommand(WristPosition.CORAL_L1);
  }

  /**
   * @return Command to move the wrist to coral L1 back scoring position
   */
  public final Command coralL1Back() {
    return setPositionCommand(WristPosition.CORAL_L1BACK);
  }

  /**
   * @return Command to move the wrist to coral L2 scoring position
   */
  public final Command coralL2() {
    return setPositionCommand(WristPosition.CORAL_L2);
  }

  /**
   * @return Command to move the wrist to coral L2 back scoring position
   */
  public final Command coralL2Back() {
    return setPositionCommand(WristPosition.CORAL_L2BACK);
  }

  /**
   * @return Command to move the wrist to coral L3 scoring position
   */
  public final Command coralL3() {
    return setPositionCommand(WristPosition.CORAL_L3);
  }

  /**
   * @return Command to move the wrist to coral L3 back scoring position
   */
  public final Command coralL3Back() {
    return setPositionCommand(WristPosition.CORAL_L3BACK);
  }

  /**
   * @return Command to move the wrist to coral L4 back scoring position
   */
  public final Command coralL4Back() {
    return setPositionCommand(WristPosition.CORAL_L4BACK);
  }

  /**
   * @return Command to move the wrist to algae floor intake position
   */
  public final Command algaeFloorIntake() {
    return setPositionCommand(WristPosition.ALGAE_FLOOR_INTAKE);
  }

  /**
   * @return Command to move the wrist to algae score position
   */
  public final Command algaeScore() {
    return setPositionCommand(WristPosition.ALGAE_SCORE);
  }

  /**
   * @return Command to move the wrist to algae L1 position
   */
  public final Command algaeL1() {
    return setPositionCommand(WristPosition.ALGAE_L1);
  }

  /**
   * @return Command to move the wrist to algae L2 position
   */
  public final Command algaeL2() {
    return setPositionCommand(WristPosition.ALGAE_L2);
  }

  /**
   * @return Command to move the wrist to climb position
   */
  public final Command climb() {
    return setPositionCommand(WristPosition.CLIMB);
  }

  /**
   * @return Command to move the wrist to climb down position
   */
  public final Command climbDown() {
    return setPositionCommand(WristPosition.CLIMB_DOWN);
  }

  @AutoLogOutput(key = "Wrist/BrakeMode")
  public boolean getBrakeMode() {
    return inputs.brakeMode;
  }

  public void setZeroed(boolean zero) {
    this.zeroed = zero;
  }

  public boolean isZeroed() {
    return zeroed;
  }

  /**
   * Sets the wrist motors to either brake or coast mode.
   *
   * @param brake True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  /** Toggles between brake and coast mode. */
  public void toggleBrakeMode() {
    setBrakeMode(!getBrakeMode());
  }
}
