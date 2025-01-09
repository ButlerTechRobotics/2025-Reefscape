// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.arm_extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The ArmExtension subsystem controls a dual-motor elevator mechanism for extending/retracting the
 * arm. It supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class ArmExtension extends SubsystemBase {
  // Hardware interface and inputs
  private final ArmExtensionIO io;
  private final ArmExtensionIOInputsAutoLogged inputs;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Extension leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Extension follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert =
      new Alert("Extension encoder isn't connected", AlertType.kError);

  // System identification routine configuration
  private final SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              state -> Logger.recordOutput("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                this.runVoltage(output);
                Logger.recordOutput("Extension_Position", output.in(Volts));
              },
              null,
              this));

  // Current extension position mode
  private ExtensionPosition currentMode = ExtensionPosition.RETRACTED;

  /**
   * Creates a new ArmExtension subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the extension
   */
  public ArmExtension(ArmExtensionIO io) {
    this.io = io;
    this.inputs = new ArmExtensionIOInputsAutoLogged();
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
   * Runs the extension in open-loop mode at the specified voltage.
   *
   * @param volts The voltage to apply to the motors
   */
  public void runVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  /**
   * Runs the extension in closed-loop position mode to the specified distance.
   *
   * @param position The target extension distance
   */
  public void setPosition(Distance position) {
    io.setPosition(position);
  }

  /** Stops the extension motors. */
  public void stop() {
    io.stop();
  }

  /**
   * Returns a command to run a quasistatic system identification test.
   *
   * @param direction The direction to run the test
   * @return The command to run the test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Returns a command to run a dynamic system identification test.
   *
   * @param direction The direction to run the test
   * @return The command to run the test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /**
   * Returns the current position of the extension.
   *
   * @return The current extension distance
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /** Enumeration of available extension positions with their corresponding target distances. */
  public enum ExtensionPosition {
    STOP(Meters.of(0)), // Stop the extension
    RETRACTED(Meters.of(0)), // Fully retracted
    PARTIAL(Meters.of(0.5)), // Partially extended for certain game pieces
    EXTENDED(Meters.of(1.0)), // Fully extended for scoring
    AMP(Meters.of(0.75)); // Specific position for amp scoring

    private final Distance targetDistance;
    private final double distanceTolerance;

    ExtensionPosition(Distance targetDistance, double distanceTolerance) {
      this.targetDistance = targetDistance;
      this.distanceTolerance = distanceTolerance;
    }

    ExtensionPosition(Distance targetDistance) {
      this(targetDistance, 0.02); // 2cm default tolerance
    }
  }

  /**
   * Gets the current extension position mode.
   *
   * @return The current ExtensionPosition
   */
  public ExtensionPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new extension position and schedules the corresponding command.
   *
   * @param position The desired ExtensionPosition
   */
  public void setExtensionPosition(ExtensionPosition position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ExtensionPosition.STOP,
              Commands.runOnce(this::stop)
                  .alongWith(Commands.run(() -> checkAtTarget(ExtensionPosition.STOP)))
                  .withName("Stop Extension"),
              ExtensionPosition.RETRACTED,
              createPositionCommand(ExtensionPosition.RETRACTED),
              ExtensionPosition.PARTIAL,
              createPositionCommand(ExtensionPosition.PARTIAL),
              ExtensionPosition.EXTENDED,
              createPositionCommand(ExtensionPosition.EXTENDED),
              ExtensionPosition.AMP,
              createPositionCommand(ExtensionPosition.AMP)),
          this::getMode);

  /**
   * Creates a command for a specific extension position that moves the mechanism and checks the
   * target position.
   *
   * @param position The extension position to create a command for
   * @return A command that implements the extension movement
   */
  private Command createPositionCommand(ExtensionPosition position) {
    return Commands.parallel(
        Commands.runOnce(() -> setPosition(position.targetDistance))
            .withName("Move to " + position.toString()),
        Commands.run(() -> checkAtTarget(position))
            .withName("Check " + position.toString() + " Target"));
  }

  /**
   * Checks if the extension is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  private boolean isAtTarget() {
    if (currentMode == ExtensionPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetDistance, currentMode.distanceTolerance);
  }

  /**
   * Logs whether the extension is at its target position for a given mode.
   *
   * @param position The position to check against
   */
  private void checkAtTarget(ExtensionPosition position) {
    boolean atTarget = isAtTarget();
    Logger.recordOutput("Extension/AtTarget", atTarget);
    Logger.recordOutput("Extension/TargetDistance", position.targetDistance);
  }

  /**
   * Creates a command to set the extension to a specific position.
   *
   * @param position The desired extension position
   * @return Command to set the position
   */
  public Command setPositionCommand(ExtensionPosition position) {
    return Commands.runOnce(() -> setExtensionPosition(position))
        .withName("SetExtensionPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to fully retract the extension
   */
  public Command retract() {
    return setPositionCommand(ExtensionPosition.RETRACTED);
  }

  /**
   * @return Command to partially extend
   */
  public Command partial() {
    return setPositionCommand(ExtensionPosition.PARTIAL);
  }

  /**
   * @return Command to fully extend
   */
  public Command extend() {
    return setPositionCommand(ExtensionPosition.EXTENDED);
  }

  /**
   * @return Command to move to amp scoring position
   */
  public Command amp() {
    return setPositionCommand(ExtensionPosition.AMP);
  }

  /**
   * @return Command to stop the extension
   */
  public Command stopCommand() {
    return setPositionCommand(ExtensionPosition.STOP);
  }
}
