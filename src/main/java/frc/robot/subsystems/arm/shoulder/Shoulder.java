// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.shoulder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Shoulder subsystem controls a quad-motor shoulder mechanism for game piece manipulation. It
 * supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Shoulder extends SubsystemBase {
  // Homing parameters
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Shoulder/HomingVolts", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Shoulder/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Shoulder/HomingVelocityThresh", 5.0);

  // Hardware interface and inputs
  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs;

  // Current shoulder position mode
  private ShoulderPosition currentMode = ShoulderPosition.STOW;

  // Alerts for motor connection status
  private final Alert brLeaderMotorAlert =
      new Alert("Shoulder back-right leader motor isn't connected", AlertType.kError);
  private final Alert blFollowerMotorAlert =
      new Alert("Shoulder back-left follower motor isn't connected", AlertType.kError);
  private final Alert frFollowerMotorAlert =
      new Alert("Shoulder front-right follower motor isn't connected", AlertType.kError);
  private final Alert flFollowerMotorAlert =
      new Alert("Shoulder front-left follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Shoulder encoder isn't connected", AlertType.kError);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  private boolean stopProfile = false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  @AutoLogOutput(key = "Shoulder/HomedPositionRot")
  private double homedPosition = 0.0;

  @AutoLogOutput(key = "Shoulder/Homed")
  private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

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
    brLeaderMotorAlert.set(!inputs.brLeaderConnected);
    blFollowerMotorAlert.set(!inputs.blFollowerConnected);
    frFollowerMotorAlert.set(!inputs.frFollowerConnected);
    flFollowerMotorAlert.set(!inputs.flFollowerConnected);
    encoderAlert.set(!inputs.encoderConnected);

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Log state
    Logger.recordOutput("Shoulder/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Shoulder/DisabledOverride", disabledOverride.getAsBoolean());
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
   * @param angle The desired angle in Rotations
   */
  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  /**
   * Creates a command to set the shoulder to a specific angle.
   *
   * @param angle The desired angle in Rotations
   * @return Command to set the angle
   */
  public Command setAngleCommand(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle))
        .withName("SetShoulderAngle(" + angle.in(Rotations) + ")");
  }

  /** Enumeration of available shoulder positions with their corresponding target angles. */
  public enum ShoulderPosition {
    // Common positions
    STOP(Rotations.of(0)),
    STOW(Rotations.of(0)),
    STANDBY(Rotations.of(0.6)),
    CLIMB(Rotations.of(1.5)),

    // Coral positions
    CORAL_FLOOR_INTAKE(Rotations.of(0.1)),
    CORAL_STATION_INTAKE(Rotations.of(0.98)),
    CORAL_L1(Rotations.of(0)),
    CORAL_L1BACK(Rotations.of(0)),
    CORAL_L2(Rotations.of(0)),
    CORAL_L2BACK(Rotations.of(0)),
    CORAL_L3(Rotations.of(0)),
    CORAL_L3BACK(Rotations.of(1.369)),
    CORAL_L4BACK(Rotations.of(0)),

    // Algae positions
    ALGAE_FLOOR_INTAKE(Rotations.of(0)),
    ALGAE_SCORE(Rotations.of(0)),
    ALGAE_L1(Rotations.of(0)),
    ALGAE_L2(Rotations.of(0));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ShoulderPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ShoulderPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2.0)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current shoulder position mode.
   *
   * @return The current ShoulderPosition
   */
  @AutoLogOutput(key = "Shoulder/ActivePosition")
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

  /**
   * Sets override suppliers for coast mode and disabled state.
   *
   * <p>These overrides provide safety controls to:
   *
   * <ul>
   *   <li>coastOverride: When true, puts motors in coast mode regardless of normal operation state
   *   <li>disabledOverride: When true, prevents motors from being driven even when the robot is
   *       enabled
   * </ul>
   *
   * <p>Typically connected to operator controls for manual safety cutoffs.
   *
   * @param coastOverride Supplier that returns true when coast mode should be forced
   * @param disabledOverride Supplier that returns true when motors should be disabled
   */
  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  /**
   * Sets the brake mode on all shoulder motors.
   *
   * <p>In brake mode, motors actively resist movement when not powered. In coast mode, motors spin
   * freely when not powered.
   *
   * <p>Only changes mode when the requested state differs from current state to minimize CAN bus
   * traffic and wear on the motor controllers.
   *
   * @param enabled true for brake mode, false for coast mode
   */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  /**
   * Sets the current position of the shoulder as the "home" or zero position.
   *
   * <p>Called at the end of the homing sequence when the shoulder has reached its mechanical limit.
   * This position becomes the reference point for all future shoulder movements.
   *
   * <p>Records the absolute position in rotations and marks the shoulder as homed.
   */
  public void setHome() {
    homedPosition = inputs.shoulderAngle.abs(Rotations);
    homed = true;
  }

  /**
   * Creates a command that homes the shoulder by driving it against a mechanical hard stop.
   *
   * <p>The homing sequence works as follows:
   *
   * <ol>
   *   <li>Disables any running motion profiles and resets homing state
   *   <li>Applies a constant negative voltage ({@link homingVolts}) to drive the shoulder toward
   *       its hard stop
   *   <li>Continuously monitors the shoulder's velocity while it's moving
   *   <li>When the shoulder reaches the hard stop, it will stall and velocity will drop
   *   <li>Once velocity stays below threshold ({@link homingVelocityThresh}) for the required time
   *       period ({@link homingTimeSecs}), the shoulder is considered homed
   *   <li>The current position is then calibrated as the home/zero position
   * </ol>
   *
   * <p>Safety features:
   *
   * <ul>
   *   <li>Will not run if {@code disabledOverride} or {@code coastOverride} is active
   *   <li>Uses a debouncer to ensure velocity is consistently below threshold before completing
   * </ul>
   *
   * @return A command that executes the shoulder homing sequence
   */
  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.encoderVelocity.abs(RotationsPerSecond))
                          <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(this::setHome)
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.ofEntries(
              // Common positions
              Map.entry(
                  ShoulderPosition.STOP, Commands.runOnce(this::stop).withName("Stop Shoulder")),
              Map.entry(ShoulderPosition.STOW, createPositionCommand(ShoulderPosition.STOW)),
              Map.entry(ShoulderPosition.STANDBY, createPositionCommand(ShoulderPosition.STANDBY)),
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
   * @return Command to move the shoulder to standby position
   */
  public final Command standby() {
    return setPositionCommand(ShoulderPosition.STANDBY);
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
