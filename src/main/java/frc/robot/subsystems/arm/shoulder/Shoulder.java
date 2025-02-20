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
    // Common positions
    STOP(Degrees.of(0)),
    STOW(Degrees.of(0)),
    CLIMB(Degrees.of(90)),

    // Coral positions
    CORAL_FLOOR_INTAKE(Degrees.of(10)),
    CORAL_STATION_INTAKE(Degrees.of(45)),
    CORAL_L1(Degrees.of(45)),
    CORAL_L1BACK(Degrees.of(90)),
    CORAL_L2(Degrees.of(25)),
    CORAL_L2BACK(Degrees.of(90)),
    CORAL_L3(Degrees.of(25)),
    CORAL_L3BACK(Degrees.of(90)),
    CORAL_L4BACK(Degrees.of(90)),

    // Algae positions
    ALGAE_FLOOR_INTAKE(Degrees.of(0)),
    ALGAE_SCORE(Degrees.of(0)),
    ALGAE_L1(Degrees.of(0)),
    ALGAE_L2(Degrees.of(0));

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
              // Common positions
              Map.entry(
                  ShoulderPosition.STOP, Commands.runOnce(this::stop).withName("Stop Shoulder")),
              Map.entry(ShoulderPosition.STOW, createPositionCommand(ShoulderPosition.STOW)),
              Map.entry(ShoulderPosition.CLIMB, createPositionCommand(ShoulderPosition.CLIMB)),

              // Coral positions
              Map.entry(
                  ShoulderPosition.CORAL_FLOOR_INTAKE,
                  createPositionCommand(ShoulderPosition.CORAL_FLOOR_INTAKE)),
              Map.entry(
                  ShoulderPosition.CORAL_STATION_INTAKE,
                  createPositionCommand(ShoulderPosition.CORAL_STATION_INTAKE)),
              Map.entry(
                  ShoulderPosition.CORAL_L1, createPositionCommand(ShoulderPosition.CORAL_L1)),
              Map.entry(
                  ShoulderPosition.CORAL_L1BACK,
                  createPositionCommand(ShoulderPosition.CORAL_L1BACK)),
              Map.entry(
                  ShoulderPosition.CORAL_L2, createPositionCommand(ShoulderPosition.CORAL_L2)),
              Map.entry(
                  ShoulderPosition.CORAL_L2BACK,
                  createPositionCommand(ShoulderPosition.CORAL_L2BACK)),
              Map.entry(
                  ShoulderPosition.CORAL_L3, createPositionCommand(ShoulderPosition.CORAL_L3)),
              Map.entry(
                  ShoulderPosition.CORAL_L3BACK,
                  createPositionCommand(ShoulderPosition.CORAL_L3BACK)),
              Map.entry(
                  ShoulderPosition.CORAL_L4BACK,
                  createPositionCommand(ShoulderPosition.CORAL_L4BACK)),

              // Algae positions
              Map.entry(
                  ShoulderPosition.ALGAE_FLOOR_INTAKE,
                  createPositionCommand(ShoulderPosition.ALGAE_FLOOR_INTAKE)),
              Map.entry(
                  ShoulderPosition.ALGAE_SCORE,
                  createPositionCommand(ShoulderPosition.ALGAE_SCORE)),
              Map.entry(
                  ShoulderPosition.ALGAE_L1, createPositionCommand(ShoulderPosition.ALGAE_L1)),
              Map.entry(
                  ShoulderPosition.ALGAE_L2, createPositionCommand(ShoulderPosition.ALGAE_L2))),
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
   * @return Command to move the shoulder to coral floor intake position
   */
  public final Command coralFloorIntake() {
    return setPositionCommand(ShoulderPosition.CORAL_FLOOR_INTAKE);
  }

  /**
   * @return Command to move the shoulder to coral station intake position
   */
  public final Command coralStationIntake() {
    return setPositionCommand(ShoulderPosition.CORAL_STATION_INTAKE);
  }

  /**
   * @return Command to move the shoulder to coral L1 scoring position
   */
  public final Command coralL1() {
    return setPositionCommand(ShoulderPosition.CORAL_L1);
  }

  /**
   * @return Command to move the shoulder to coral L1 back scoring position
   */
  public final Command coralL1Back() {
    return setPositionCommand(ShoulderPosition.CORAL_L1BACK);
  }

  /**
   * @return Command to move the shoulder to coral L2 scoring position
   */
  public final Command coralL2() {
    return setPositionCommand(ShoulderPosition.CORAL_L2);
  }

  /**
   * @return Command to move the shoulder to coral L2 back scoring position
   */
  public final Command coralL2Back() {
    return setPositionCommand(ShoulderPosition.CORAL_L2BACK);
  }

  /**
   * @return Command to move the shoulder to coral L3 scoring position
   */
  public final Command coralL3() {
    return setPositionCommand(ShoulderPosition.CORAL_L3);
  }

  /**
   * @return Command to move the shoulder to coral L3 back scoring position
   */
  public final Command coralL3Back() {
    return setPositionCommand(ShoulderPosition.CORAL_L3BACK);
  }

  /**
   * @return Command to move the shoulder to coral L4 back scoring position
   */
  public final Command coralL4Back() {
    return setPositionCommand(ShoulderPosition.CORAL_L4BACK);
  }

  /**
   * @return Command to move the shoulder to algae floor intake position
   */
  public final Command algaeFloorIntake() {
    return setPositionCommand(ShoulderPosition.ALGAE_FLOOR_INTAKE);
  }

  /**
   * @return Command to move the shoulder to algae score position
   */
  public final Command algaeScore() {
    return setPositionCommand(ShoulderPosition.ALGAE_SCORE);
  }

  /**
   * @return Command to move the shoulder to algae L1 position
   */
  public final Command algaeL1() {
    return setPositionCommand(ShoulderPosition.ALGAE_L1);
  }

  /**
   * @return Command to move the shoulder to algae L2 position
   */
  public final Command algaeL2() {
    return setPositionCommand(ShoulderPosition.ALGAE_L2);
  }

  /**
   * @return Command to move the shoulder to climb position
   */
  public final Command climb() {
    return setPositionCommand(ShoulderPosition.CLIMB);
  }
}
