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

  /** Enumeration of available wrist positions with their corresponding target angles. */
  public enum WristPosition {
    STOP(Degrees.of(0)), // Stop the arm
    STOW(Degrees.of(90)), // Stow the arm
    FLOOR_INTAKE(Degrees.of(0)), // Position for intaking from floor
    SOURCE_INTAKE(Degrees.of(0)), // Position for intaking from source
    L1(Degrees.of(-90)), // Position for scoring in L1
    L1Back(Degrees.of(130)), // Position for scoring in L1Back
    L2(Degrees.of(-90)), // Position for scoring in L2
    L2Back(Degrees.of(110)), // Position for scoring in L2Back
    L3(Degrees.of(-90)), // Position for scoring in L3
    L3Back(Degrees.of(110)), // Position for scoring in L3Back
    L4(Degrees.of(-90)), // Position for scoring in L4
    L4Back(Degrees.of(130)), // Position for scoring in L4Back
    CLIMB(Degrees.of(-90)); // Position for climbing

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
              Map.entry(WristPosition.STOP, Commands.runOnce(this::stop).withName("Stop Wrist")),
              Map.entry(WristPosition.STOW, createPositionCommand(WristPosition.STOW)),
              Map.entry(
                  WristPosition.FLOOR_INTAKE, createPositionCommand(WristPosition.FLOOR_INTAKE)),
              Map.entry(
                  WristPosition.SOURCE_INTAKE, createPositionCommand(WristPosition.SOURCE_INTAKE)),
              Map.entry(WristPosition.L1, createPositionCommand(WristPosition.L1)),
              Map.entry(WristPosition.L1Back, createPositionCommand(WristPosition.L1Back)),
              Map.entry(WristPosition.L2, createPositionCommand(WristPosition.L2)),
              Map.entry(WristPosition.L2Back, createPositionCommand(WristPosition.L2Back)),
              Map.entry(WristPosition.L3, createPositionCommand(WristPosition.L3)),
              Map.entry(WristPosition.L3Back, createPositionCommand(WristPosition.L3Back)),
              Map.entry(WristPosition.L4, createPositionCommand(WristPosition.L4)),
              Map.entry(WristPosition.L4Back, createPositionCommand(WristPosition.L4Back)),
              Map.entry(WristPosition.CLIMB, createPositionCommand(WristPosition.CLIMB))),
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
   * @return Command to move the wrist to floor intake position
   */
  public final Command intake() {
    return setPositionCommand(WristPosition.FLOOR_INTAKE);
  }

  /**
   * @return Command to move the wrist to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(WristPosition.L1);
  }

  /**
   * @return Command to move the wrist to L1Back scoring position
   */
  public final Command L1Back() {
    return setPositionCommand(WristPosition.L1Back);
  }

  /**
   * @return Command to move the wrist to L2 scoring position
   */
  public final Command L2() {
    return setPositionCommand(WristPosition.L2);
  }

  /**
   * @return Command to move the wrist to L2Back scoring position
   */
  public final Command L2Back() {
    return setPositionCommand(WristPosition.L2Back);
  }

  /**
   * @return Command to move the wrist to L3 scoring position
   */
  public final Command L3() {
    return setPositionCommand(WristPosition.L3);
  }

  /**
   * @return Command to move the wrist to L3Back scoring position
   */
  public final Command L3Back() {
    return setPositionCommand(WristPosition.L3Back);
  }

  /**
   * @return Command to move the wrist to L4 scoring position
   */
  public final Command L4() {
    return setPositionCommand(WristPosition.L4);
  }

  /**
   * @return Command to move the wrist to L4Back scoring position
   */
  public final Command L4Back() {
    return setPositionCommand(WristPosition.L4Back);
  }

  /**
   * @return Command to move the wrist to climb position
   */
  public final Command climb() {
    return setPositionCommand(WristPosition.CLIMB);
  }
}
