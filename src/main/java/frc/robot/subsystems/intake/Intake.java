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
  private final BooleanSupplier isShufflingTrigger = this::isShuffling;

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
    } else if (hasFront && !hasBack) {
      currentCoralState = CoralState.FRONT;
    } else if (!hasFront && hasBack) {
      currentCoralState = CoralState.BACK;
    } else {
      currentCoralState = CoralState.SHUFFLING;
    }
  }

  /** Stops the intake motor. */
  public void stop() {
    io.stop();
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
    STOP(Voltage.ofBaseUnits(0, Volts)), // Voltage for stopping the intake
    CORAL_INTAKE(Voltage.ofBaseUnits(5, Volts)), // Voltage for coral intake
    CORAL_SHUFFLE_IN(
        Voltage.ofBaseUnits(1, Volts)), // Voltage for shuffling coral from front to back
    CORAL_SHUFFLE_OUT(
        Voltage.ofBaseUnits(-1, Volts)), // Voltage for shuffling coral from back to front
    ALGAE_INTAKE(Voltage.ofBaseUnits(12, Volts)), // Voltage for algae intake
    SHOOT_FRONT(Voltage.ofBaseUnits(-5, Volts)), // Voltage for shooting a game piece out the front
    SHOOT_BACK(Voltage.ofBaseUnits(5, Volts)); // Voltage for shooting a game piece out the back

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
    SHUFFLING // Coral detected at both sensors (transitioning)
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
   * Checks if the intake has a game piece detected by both sensors (shuffling).
   *
   * @return true if a game piece is detected by both sensors, false otherwise
   */
  @AutoLogOutput
  public boolean isShuffling() {
    return inputs.hasFrontGamePiece && inputs.hasBackGamePiece;
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
   * Creates a command that waits until a game piece is detected at the front sensor.
   *
   * @return A command that waits for front game piece detection
   */
  public Command waitForFrontGamePiece() {
    return Commands.waitUntil(hasFrontGamePieceTrigger);
  }

  /**
   * Creates a command that waits until a game piece is detected at the back sensor.
   *
   * @return A command that waits for back game piece detection
   */
  public Command waitForBackGamePiece() {
    return Commands.waitUntil(hasBackGamePieceTrigger);
  }

  /**
   * Creates a command that waits until the game piece is in a shuffling state.
   *
   * @return A command that waits for shuffling state
   */
  public Command waitForShuffling() {
    return Commands.waitUntil(isShufflingTrigger);
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
   * Sets the intake to a new mode if different from the current mode.
   *
   * @param mode The new IntakeMode to set
   */
  private void setIntakeMode(IntakeMode mode) {
    if (currentMode != mode) {
      currentMode = mode;
      io.setVoltage(mode.targetVoltage);
    }
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
  public Command CORAL_INTAKE() {
    return setIntakeStateCommand(IntakeMode.CORAL_INTAKE);
  }

  /**
   * Creates a command to set the wheels to shuffle coral from front to back.
   *
   * @return A command that sets the wheels to shuffle coral
   */
  public Command CORAL_SHUFFLE_IN() {
    return setIntakeStateCommand(IntakeMode.CORAL_SHUFFLE_IN);
  }

  /**
   * Creates a command to set the wheels to shuffle coral from back to front.
   *
   * @return A command that sets the wheels to shuffle coral
   */
  public Command CORAL_SHUFFLE_OUT() {
    return setIntakeStateCommand(IntakeMode.CORAL_SHUFFLE_OUT);
  }

  /**
   * Creates a command to set the wheels to intake algae.
   *
   * @return A command that sets the wheels to intake algae
   */
  public Command ALGAE_INTAKE() {
    return setIntakeStateCommand(IntakeMode.ALGAE_INTAKE);
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
    return setIntakeStateCommand(IntakeMode.SHOOT_FRONT);
  }

  /**
   * Creates a command to automatically outtake a game piece.
   *
   * @return A command that outtakes a game piece
   */
  public Command AUTO_OUTTAKE() {
    return Commands.runOnce(() -> io.setVoltage(Volts.of(-3)))
        .andThen(gamePieceLoaded())
        .andThen(waitForNoGamePiece())
        .andThen(STOP());
  }

  /**
   * Creates a command to stop the intake.
   *
   * @return A command that stops the intake
   */
  public Command STOP() {
    return Commands.runOnce(this::stop, this).withName("Stop");
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

  /**
   * Creates a command to intake a coral to the back position.
   *
   * @return A command that intakes a coral to the back position
   */
  public Command intakeCoralToBack() {
    return CORAL_INTAKE()
        .andThen(waitForBackGamePiece())
        .andThen(STOP())
        .withName("Intake Coral To Back");
  }

  /**
   * Creates a command to intake a coral to the front position.
   *
   * @return A command that intakes a coral to the front position
   */
  public Command intakeCoralToFront() {
    return CORAL_INTAKE()
        .andThen(waitForFrontGamePiece())
        .andThen(STOP())
        .withName("Intake Coral To Front");
  }

  /**
   * Creates a command to shuffle a coral from the front to the back.
   *
   * @return A command that shuffles a coral from front to back
   */
  public Command shuffleCoralToBack() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (currentCoralState != CoralState.FRONT) {
                    System.out.println(
                        "Warning: Attempting to shuffle from front when coral is not at front");
                  }
                }),
            CORAL_SHUFFLE_IN(),
            waitForBackGamePiece(),
            STOP())
        .withName("Shuffle Coral To Back");
  }

  /**
   * Creates a command to shuffle a coral from the back to the front.
   *
   * @return A command that shuffles a coral from back to front
   */
  public Command shuffleCoralToFront() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (currentCoralState != CoralState.BACK) {
                    System.out.println(
                        "Warning: Attempting to shuffle from back when coral is not at back");
                  }
                }),
            CORAL_SHUFFLE_OUT(),
            waitForFrontGamePiece(),
            STOP())
        .withName("Shuffle Coral To Front");
  }

  /**
   * Creates a command to score a coral from the front position.
   *
   * @return A command that scores a coral from the front
   */
  public Command scoreCoralFromFront() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (currentCoralState != CoralState.FRONT) {
                    System.out.println(
                        "Warning: Attempting to score from front when coral is not at front");
                  }
                }),
            SHOOT_FRONT(),
            waitForNoGamePiece(),
            STOP())
        .withName("Score Coral From Front");
  }

  /**
   * Creates a command to score a coral from the back position.
   *
   * @return A command that scores a coral from the back
   */
  public Command scoreCoralFromBack() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (currentCoralState != CoralState.BACK) {
                    System.out.println(
                        "Warning: Attempting to score from back when coral is not at back");
                  }
                }),
            SHOOT_BACK(),
            waitForNoGamePiece(),
            STOP())
        .withName("Score Coral From Back");
  }
}
