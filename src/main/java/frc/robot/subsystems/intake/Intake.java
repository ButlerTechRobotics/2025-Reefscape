// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Intake subsystem controls a motor-driven intake mechanism for game piece manipulation. It
 * supports multiple speeds for different game actions and provides open-loop control.
 */
public class Intake extends SubsystemBase {
  // Hardware interface and inputs
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private IntakeMode currentMode;
  private CoralState currentCoralState;

  // Alerts for motor and sensor connection status
  private final Alert motorAlert = new Alert("Intake motor isn't connected", AlertType.kError);
  private final Alert frontCANrangeAlert =
      new Alert("Intake front CANrange isn't connected", AlertType.kError);
  private final Alert backCANrangeAlert =
      new Alert("Intake back CANrange isn't connected", AlertType.kError);

  // Triggers for game piece detection
  private final BooleanSupplier hasGamePieceTrigger = this::hasGamePiece;
  private final BooleanSupplier hasFrontGamePieceTrigger = this::hasFrontGamePiece;
  private final BooleanSupplier hasBackGamePieceTrigger = this::hasBackGamePiece;

  /**
   * Creates a new Intake subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the intake
   */
  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
    this.currentMode = IntakeMode.STOP;
    this.currentCoralState = CoralState.NONE;
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update motor connection status alerts
    motorAlert.set(!inputs.leaderConnected);
    frontCANrangeAlert.set(!inputs.frontCANrangeConnected);
    backCANrangeAlert.set(!inputs.backCANrangeConnected);

    // Update coral state based on sensor readings
    updateCoralState();
  }

  /** Updates the coral state based on sensor readings. */
  private void updateCoralState() {
    boolean hasFront = inputs.hasFrontGamePiece;
    boolean hasBack = inputs.hasBackGamePiece;

    if (!hasFront && !hasBack) {
      currentCoralState = CoralState.NONE;
    } else if (!hasFront && hasBack) {
      currentCoralState = CoralState.FRONT;
    } else if (hasFront && !hasBack) {
      currentCoralState = CoralState.BACK;
    } else {
      // Both sensors detect a game piece
      currentCoralState = CoralState.BOTH;
    }
  }

  /** Stops the intake motor. */
  public void stop() {
    io.stop();
    currentMode = IntakeMode.STOP;
  }

  /**
   * Returns the current voltage of the intake.
   *
   * @return The current voltage
   */
  @AutoLogOutput
  public Voltage getVoltage() {
    return inputs.appliedVoltage;
  }

  /** Enumeration of available intake positions with their corresponding target angles. */
  public enum IntakeMode {
    STOP(Voltage.ofBaseUnits(0, Volts)),
    INTAKE(Voltage.ofBaseUnits(8, Volts)),
    SHUFFLE_TO_FRONT(Voltage.ofBaseUnits(-1.5, Volts)),
    SHUFFLE_TO_BACK(Voltage.ofBaseUnits(1.5, Volts)),
    SHOOT_FRONT(Voltage.ofBaseUnits(-5, Volts)),
    SHOOT_BACK(Voltage.ofBaseUnits(5, Volts));

    private final Voltage targetVoltage;

    /**
     * Creates a new IntakeMode with specified target voltage.
     *
     * @param targetVoltage The target voltage for this mode
     */
    IntakeMode(Voltage targetVoltage) {
      this.targetVoltage = targetVoltage;
    }
  }

  /** Enumeration of coral states within the intake. */
  public enum CoralState {
    NONE, // No coral detected
    FRONT, // Coral detected at front sensor only
    BACK, // Coral detected at back sensor only
    BOTH // Coral detected at both sensors
  }

  /**
   * Gets the current mode of the intake.
   *
   * @return The current IntakeMode
   */
  public IntakeMode getMode() {
    return currentMode;
  }

  /**
   * Gets the current state of the coral in the intake.
   *
   * @return The current CoralState
   */
  @AutoLogOutput
  public CoralState getCoralState() {
    return currentCoralState;
  }

  /**
   * Gets the target voltage for the current mode.
   *
   * @return The target voltage for the current mode
   */
  @AutoLogOutput
  public Voltage targetVoltage() {
    return currentMode.targetVoltage;
  }

  /**
   * Checks if the intake has a game piece detected by either sensor.
   *
   * @return true if a game piece is detected, false otherwise
   */
  @AutoLogOutput
  public boolean hasGamePiece() {
    return inputs.hasGamePiece;
  }

  /**
   * Checks if the intake has a game piece detected by the front sensor.
   *
   * @return true if a game piece is detected at the front, false otherwise
   */
  @AutoLogOutput
  public boolean hasFrontGamePiece() {
    return inputs.hasFrontGamePiece;
  }

  /**
   * Checks if the intake has a game piece detected by the back sensor.
   *
   * @return true if a game piece is detected at the back, false otherwise
   */
  @AutoLogOutput
  public boolean hasBackGamePiece() {
    return inputs.hasBackGamePiece;
  }

  /**
   * Creates a command that waits until a game piece is detected by either sensor.
   *
   * @return A command that waits for game piece detection
   */
  public Command waitForGamePiece() {
    return Commands.waitUntil(hasGamePieceTrigger);
  }

  /**
   * Creates a command that waits until no game piece is detected.
   *
   * @return A command that waits until no game piece is detected
   */
  public Command waitForNoGamePiece() {
    return Commands.waitUntil(() -> !hasGamePiece());
  }

  /**
   * Creates a command that waits until a game piece is detected at the front sensor only.
   *
   * @return A command that waits for front-only game piece detection
   */
  public Command waitForFrontGamePiece() {
    return Commands.waitUntil(hasFrontGamePieceTrigger);
  }

  /**
   * Creates a command that waits until a game piece is detected at the back sensor only.
   *
   * @return A command that waits for back-only game piece detection
   */
  public Command waitForBackGamePiece() {
    return Commands.waitUntil(hasBackGamePieceTrigger);
  }

  /**
   * Gets the current distance reading from the front CANrange.
   *
   * @return The current front CANrange distance
   */
  @AutoLogOutput
  private Distance frontCANrangeDistance() {
    return inputs.frontCANrangeDistance;
  }

  /**
   * Gets the current distance reading from the back CANrange.
   *
   * @return The current back CANrange distance
   */
  @AutoLogOutput
  private Distance backCANrangeDistance() {
    return inputs.backCANrangeDistance;
  }

  /**
   * Sets the intake to a new mode and applies the corresponding voltage.
   *
   * @param mode The new IntakeMode to set
   */
  private void setIntakeMode(IntakeMode mode) {
    currentMode = mode;
    io.setVoltage(mode.targetVoltage);
  }

  /**
   * Creates a command that sets the intake to a specified mode.
   *
   * @param mode The IntakeMode to set
   * @return A command that sets the intake mode
   */
  public Command setIntakeStateCommand(IntakeMode mode) {
    return Commands.runOnce(() -> setIntakeMode(mode), this).withName("Move to " + mode.toString());
  }

  /**
   * Creates a command to set the wheels to intake coral.
   *
   * @return A command that sets the wheels to intake coral
   */
  public Command INTAKE() {
    return setIntakeStateCommand(IntakeMode.INTAKE);
  }

  /**
   * Creates a command to set the wheels to shuffle coral toward front.
   *
   * @return A command that sets the wheels to shuffle toward front
   */
  public Command SHUFFLE_TO_FRONT() {
    return setIntakeStateCommand(IntakeMode.SHUFFLE_TO_FRONT);
  }

  /**
   * Creates a command to set the wheels to shuffle coral toward back.
   *
   * @return A command that sets the wheels to shuffle toward back
   */
  public Command SHUFFLE_TO_BACK() {
    return setIntakeStateCommand(IntakeMode.SHUFFLE_TO_BACK);
  }

  /**
   * Creates a command to set the wheels to shoot a game piece out the front of the intake.
   *
   * @return A command that sets the wheels to shoot front
   */
  public Command SHOOT_FRONT() {
    return setIntakeStateCommand(IntakeMode.SHOOT_FRONT);
  }

  /**
   * Creates a command to set the wheels to shoot a game piece out the back of the intake.
   *
   * @return A command that sets the wheels to shoot back
   */
  public Command SHOOT_BACK() {
    return setIntakeStateCommand(IntakeMode.SHOOT_BACK);
  }

  /**
   * Creates a command to stop the intake.
   *
   * @return A command that stops the intake
   */
  public Command STOP() {
    return setIntakeStateCommand(IntakeMode.STOP);
  }

  /**
   * Creates a command to intake a coral until it's detected by the back sensor.
   *
   * @return A command that intakes a coral to the back position
   */
  public Command intakeCoralToBack() {
    return INTAKE()
        .andThen(Commands.waitUntil(hasBackGamePieceTrigger))
        .andThen(STOP())
        .withName("Intake Coral To Back");
  }

  /**
   * Creates a command to score a coral from the front position.
   *
   * @return A command that scores a coral from the front
   */
  public Command scoreCoralFromFront() {
    return Commands.sequence(SHOOT_FRONT(), waitForNoGamePiece(), STOP())
        .withName("Score Coral From Front");
  }

  /**
   * Creates a command to score a coral from the back position.
   *
   * @return A command that scores a coral from the back
   */
  public Command scoreCoralFromBack() {
    return Commands.sequence(SHOOT_BACK(), waitForNoGamePiece(), STOP())
        .withName("Score Coral From Back");
  }

  public Command AUTO_SHOOT() {
    return Commands.runOnce(() -> io.setVoltage(Volts.of(-5)))
        .andThen(gamePieceLoaded())
        .andThen(waitForNoGamePiece())
        .andThen(STOP());
  }

  public Command AUTO_INTAKE() {
    return Commands.runOnce(() -> io.setVoltage(Volts.of(8)))
        .andThen(gamePieceUnloaded())
        .andThen(waitForBackGamePiece())
        .andThen(STOP());
  }

  /**
   * Creates a command that pulses the intake at a specified voltage.
   *
   * @param mode The IntakeMode to pulse
   * @param pulseDurationSeconds How long each pulse should last in seconds
   * @param pauseDurationSeconds How long to pause between pulses in seconds
   * @return A command that pulses the intake
   */
  private Command pulseIntake(
      IntakeMode mode, double pulseDurationSeconds, double pauseDurationSeconds) {
    return Commands.sequence(
            setIntakeStateCommand(mode),
            Commands.waitSeconds(pulseDurationSeconds),
            STOP(),
            Commands.waitSeconds(pauseDurationSeconds))
        .withName("Pulse " + mode.toString());
  }

  /**
   * Creates a command to shuffle a coral to the back position using pulses. Will stop when the game
   * piece is no longer detected at the front sensor.
   *
   * @return A command that shuffles a coral to the back with pulsing
   */
  public Command shuffleCoralToBack() {
    return Commands.sequence(
            // Log start of operation
            Commands.runOnce(
                () ->
                    System.out.println(
                        "Starting shuffle to back with pulsing, current state: "
                            + currentCoralState)),

            // Repeat pulses until condition is met
            Commands.repeatingSequence(pulseIntake(IntakeMode.SHUFFLE_TO_BACK, 0.05, 0.05))
                .until(() -> !hasFrontGamePiece()),

            // Final pulse to position
            // SHUFFLE_TO_FRONT(),
            // Commands.waitSeconds(0.1),
            STOP(),

            // Log completion
            Commands.runOnce(
                () -> System.out.println("Finished shuffle to back, state: " + currentCoralState)))
        .withName("Shuffle Coral To Back")
        .withTimeout(3); // Add safety timeout
  }

  /**
   * Creates a command to shuffle a coral to the front position using pulses. Will stop when the
   * game piece is no longer detected at the back sensor.
   *
   * @return A command that shuffles a coral to the front with pulsing
   */
  public Command shuffleCoralToFront() {
    return Commands.sequence(
            // Log start of operation
            Commands.runOnce(
                () ->
                    System.out.println(
                        "Starting shuffle to front with pulsing, current state: "
                            + currentCoralState)),

            // Repeat pulses until condition is met
            Commands.repeatingSequence(pulseIntake(IntakeMode.SHUFFLE_TO_FRONT, 0.2, 0.05))
                .until(() -> !hasBackGamePiece()),

            // Final pulse to position
            SHUFFLE_TO_BACK(),
            Commands.waitSeconds(0.1),
            STOP(),

            // Log completion
            Commands.runOnce(
                () -> System.out.println("Finished shuffle to front, state: " + currentCoralState)))
        .withName("Shuffle Coral To Front")
        .withTimeout(3); // Add safety timeout
  }

  /**
   * Creates a command to set the simulated range finder distance.
   *
   * @param distance The distance to simulate
   * @return A command that sets the simulated range finder distance
   */
  private Command setCANrangeDistanceSim(Distance distance) {
    return Commands.runOnce(() -> io.setCANrangeDistanceSim(distance));
  }

  /**
   * Creates a command to simulate a game piece being loaded.
   *
   * @return A command that simulates a game piece being loaded
   */
  public Command gamePieceLoaded() {
    return setCANrangeDistanceSim(Inches.of(0.5));
  }

  /**
   * Creates a command to simulate a game piece being unloaded.
   *
   * @return A command that simulates a game piece being unloaded
   */
  public Command gamePieceUnloaded() {
    return setCANrangeDistanceSim(Inches.of(1.5));
  }
}
