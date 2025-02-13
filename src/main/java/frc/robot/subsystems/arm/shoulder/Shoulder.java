// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.shoulder;

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
 * The Shoulder subsystem controls a dual-motor shoulder mechanism for game piece manipulation. It
 * supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Shoulder extends SubsystemBase {
  // Hardware interface and inputs
  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs;

  // Current shoulder position mode
  private ShoulderPosition currentMode = ShoulderPosition.STOW;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Shoulder leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Shoulder follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Shoulder encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Shoulder subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the shoulder
   */
  public Shoulder(ShoulderIO io) {
    this.io = io;
    this.inputs = new ShoulderIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Shoulder", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the shoulder in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  private void setPosition(Angle position) {
    io.setPosition(position);
  }

  /** Stops the shoulder motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current position of the shoulder.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /**
   * Sets the shoulder to a specific angle.
   *
   * @param angle The desired angle in degrees
   */
  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  /**
   * Creates a command to set the shoulder to a specific angle.
   *
   * @param angle The desired angle in degrees
   * @return Command to set the angle
   */
  public Command setAngleCommand(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle))
        .withName("SetShoulderAngle(" + angle.in(Degrees) + ")");
  }

  /** Enumeration of available shoulder positions with their corresponding target angles. */
  public enum ShoulderPosition {
    STOP(Degrees.of(0)), // Stop the arm
    STOW(Degrees.of(0)), // Stow the arm
    FLOOR_INTAKE(Degrees.of(10)), // Position for intaking from floor
    STATION_INTAKE(Degrees.of(45)), // Position for intaking from station
    L1(Degrees.of(45)), // Position for scoring in L1
    L1Back(Degrees.of(90)), // Position for scoring in L1Back
    L2(Degrees.of(25)), // Position for scoring in L2
    L2Back(Degrees.of(90)), // Position for scoring in L2Back
    L3(Degrees.of(25)), // Position for scoring in L3
    L3Back(Degrees.of(90)), // Position for scoring in L3Back
    L4(Degrees.of(65)), // Position for scoring in L4
    L4Back(Degrees.of(90)), // Position for scoring in L4Back
    CLIMB(Degrees.of(90)); // Position for climbing

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ShoulderPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ShoulderPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current shoulder position mode.
   *
   * @return The current ShoulderPosition
   */
  public ShoulderPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new shoulder position and schedules the corresponding command.
   *
   * @param position The desired ShoulderPosition
   */
  public void setShoulderPosition(ShoulderPosition position) {
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
              Map.entry(
                  ShoulderPosition.STOP, Commands.runOnce(this::stop).withName("Stop Shoulder")),
              Map.entry(ShoulderPosition.STOW, createPositionCommand(ShoulderPosition.STOW)),
              Map.entry(
                  ShoulderPosition.FLOOR_INTAKE,
                  createPositionCommand(ShoulderPosition.FLOOR_INTAKE)),
              Map.entry(
                  ShoulderPosition.STATION_INTAKE,
                  createPositionCommand(ShoulderPosition.STATION_INTAKE)),
              Map.entry(ShoulderPosition.L1, createPositionCommand(ShoulderPosition.L1)),
              Map.entry(ShoulderPosition.L1Back, createPositionCommand(ShoulderPosition.L1Back)),
              Map.entry(ShoulderPosition.L2, createPositionCommand(ShoulderPosition.L2)),
              Map.entry(ShoulderPosition.L2Back, createPositionCommand(ShoulderPosition.L2Back)),
              Map.entry(ShoulderPosition.L3, createPositionCommand(ShoulderPosition.L3)),
              Map.entry(ShoulderPosition.L3Back, createPositionCommand(ShoulderPosition.L3Back)),
              Map.entry(ShoulderPosition.L4, createPositionCommand(ShoulderPosition.L4)),
              Map.entry(ShoulderPosition.L4Back, createPositionCommand(ShoulderPosition.L4Back)),
              Map.entry(ShoulderPosition.CLIMB, createPositionCommand(ShoulderPosition.CLIMB))),
          this::getMode);

  /**
   * Creates a command for a specific shoulder position that moves the shoulder and checks the
   * target position.
   *
   * @param position The shoulder position to create a command for
   * @return A command that implements the shoulder movement
   */
  private Command createPositionCommand(ShoulderPosition position) {
    return Commands.runOnce(() -> setPosition(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the shoulder is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ShoulderPosition.STOW) return true;
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
   * Creates a command to set the shoulder to a specific position.
   *
   * @param position The desired shoulder position
   * @return Command to set the position
   */
  private Command setPositionCommand(ShoulderPosition position) {
    return Commands.runOnce(() -> setShoulderPosition(position))
        .withName("SetShoulderPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to stop the shoulder
   */
  public final Command stopCommand() {
    return setPositionCommand(ShoulderPosition.STOP);
  }

  /**
   * @return Command to move the shoulder to stow position
   */
  public final Command stow() {
    return setPositionCommand(ShoulderPosition.STOW);
  }

  /**
   * @return Command to move the shoulder to floor intake position
   */
  public final Command floorIntake() {
    return setPositionCommand(ShoulderPosition.FLOOR_INTAKE);
  }

  /**
   * @return Command to move the shoulder to source intake position
   */
  public final Command stationIntake() {
    return setPositionCommand(ShoulderPosition.STATION_INTAKE);
  }

  /**
   * @return Command to move the shoulder to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(ShoulderPosition.L1);
  }

  /**
   * @return Command to move the shoulder to L1Back scoring position
   */
  public final Command L1Back() {
    return setPositionCommand(ShoulderPosition.L1Back);
  }

  /**
   * @return Command to move the shoulder to L2 scoring position
   */
  public final Command L2() {
    return setPositionCommand(ShoulderPosition.L2);
  }

  /**
   * @return Command to move the shoulder to L2Back scoring position
   */
  public final Command L2Back() {
    return setPositionCommand(ShoulderPosition.L2Back);
  }

  /**
   * @return Command to move the shoulder to L3 scoring position
   */
  public final Command L3() {
    return setPositionCommand(ShoulderPosition.L3);
  }

  /**
   * @return Command to move the shoulder to L3Back scoring position
   */
  public final Command L3Back() {
    return setPositionCommand(ShoulderPosition.L3Back);
  }

  /**
   * @return Command to move the shoulder to L4 scoring position
   */
  public final Command L4() {
    return setPositionCommand(ShoulderPosition.L4);
  }

  /**
   * @return Command to move the shoulder to L4Back scoring position
   */
  public final Command L4Back() {
    return setPositionCommand(ShoulderPosition.L4Back);
  }

  /**
   * @return Command to move the shoulder to climb position
   */
  public final Command climb() {
    return setPositionCommand(ShoulderPosition.CLIMB);
  }
}
