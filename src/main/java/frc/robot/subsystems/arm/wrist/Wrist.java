// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.wrist;

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
  private final Alert followerMotorAlert =
      new Alert("Wrist follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Wrist encoder isn't connected", AlertType.kError);

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
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the wrist in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  public void setPosition(Angle position) {
    io.setPosition(position);
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
   * @param angle The desired angle in degrees
   */
  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  /**
   * Creates a command to set the wrist to a specific angle.
   *
   * @param angle The desired angle in degrees
   * @return Command to set the angle
   */
  public Command setAngleCommand(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle))
        .withName("SetWristAngle(" + angle.in(Degrees) + ")");
  }

  /** Enumeration of available wrist positions with their corresponding target angles. */
  public enum WristPosition {
    // Common positions
    STOP(Degrees.of(0)),
    STOW(Degrees.of(90)),
    STANDBY(Degrees.of(90)),
    CLIMB(Degrees.of(-90)),

    // Coral positions
    CORAL_FLOOR_INTAKE(Degrees.of(0)),
    CORAL_STATION_INTAKE(Degrees.of(0)),
    CORAL_L1(Degrees.of(-90)),
    CORAL_L1BACK(Degrees.of(130)),
    CORAL_L2(Degrees.of(-90)),
    CORAL_L2BACK(Degrees.of(110)),
    CORAL_L3(Degrees.of(-90)),
    CORAL_L3BACK(Degrees.of(110)),
    CORAL_L4(Degrees.of(-90)),
    CORAL_L4BACK(Degrees.of(130)),

    // Algae positions (all 0)
    ALGAE_FLOOR_INTAKE(Degrees.of(0)),
    ALGAE_SCORE(Degrees.of(0)),
    ALGAE_L1(Degrees.of(0)),
    ALGAE_L2(Degrees.of(0));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    WristPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    WristPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2)); // 2 degree default tolerance
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

              // Coral positions
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
}
