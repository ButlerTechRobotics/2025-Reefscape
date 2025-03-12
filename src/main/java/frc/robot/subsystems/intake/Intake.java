// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
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
 * The Claw subsystem controls a motor-driven claw mechanism for game piece manipulation. It
 * supports multiple speeds for different game actions and provides open-loop control.
 */
public class Intake extends SubsystemBase {
  // Hardware interface and inputs
  private final IntakeIO io;
  private final ClawIOInputsAutoLogged inputs;

  // Alerts for motor connection status
  private final Alert motorAlert = new Alert("Claw motor isn't connected", AlertType.kError);

  // Current claw position mode
  private ClawMode currentMode = ClawMode.NONE;

  /**
   * Creates a new Claw subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the claw
   */
  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new ClawIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);

    // Update motor connection status alerts
    motorAlert.set(!inputs.leaderConnected);
  }

  /** Stops the claw motor. */
  public void stop() {
    io.stop();
  }

  /**
   * Returns the current position of the claw.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.leaderPosition;
  }

  /** Enumeration of available claw positions with their corresponding target angles. */
  public enum ClawMode {
    NONE(Voltage.ofBaseUnits(0, Volts)), // Stowed position
    FLOOR_INTAKE(Voltage.ofBaseUnits(10, Volts)), // Position for floor intake
    STATION_INTAKE(Voltage.ofBaseUnits(10, Volts)), // Position for station intake
    ALGAE_INTAKE(Voltage.ofBaseUnits(10, Volts)), // Position for algae intake
    OUTTAKE(Voltage.ofBaseUnits(-3, Volts)); // Position for gripping at level 1

    private final Voltage voltage;

    ClawMode(Voltage voltage) {
      this.voltage = voltage;
    }
  }

  /**
   * Gets the current claw position mode.
   *
   * @return The current ClawMode
   */
  public ClawMode getMode() {
    return currentMode;
  }

  /**
   * Sets a new claw position and schedules the corresponding command.
   *
   * @param position The desired ClawMode
   */
  public void setClawMode(ClawMode position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ClawMode.NONE,
              Commands.runOnce(this::stop).withName("Stop Claw"),
              ClawMode.FLOOR_INTAKE,
              createVoltageCommand(ClawMode.FLOOR_INTAKE),
              ClawMode.STATION_INTAKE,
              createVoltageCommand(ClawMode.STATION_INTAKE),
              ClawMode.ALGAE_INTAKE,
              createVoltageCommand(ClawMode.ALGAE_INTAKE),
              ClawMode.OUTTAKE,
              createVoltageCommand(ClawMode.OUTTAKE)),
          this::getMode);

  /**
   * Creates a command for a specific claw voltage that spins the claw and checks the target
   * voltage.
   *
   * @param voltage The claw voltage to create a command for
   * @return A command that implements the claw movement
   */
  private Command createVoltageCommand(ClawMode voltage) {
    return Commands.runOnce(() -> io.setVoltage(voltage.voltage))
        .withName("Set volts to " + voltage.toString());
  }

  /**
   * Creates a command to set the claw to a specific voltage.
   *
   * @param voltage The desired claw voltage
   * @return Command to set the voltage
   */
  public Command setVoltageCommand(ClawMode voltage) {
    return Commands.runOnce(() -> setClawMode(voltage))
        .withName("SetClawMode(" + voltage.toString() + ")");
  }

  /**
   * @return Command to set claw to floor intake position
   */
  public Command floorIntake() {
    return setVoltageCommand(ClawMode.FLOOR_INTAKE);
  }

  /**
   * @return Command to set claw to station intake position
   */
  public Command stationIntake() {
    return setVoltageCommand(ClawMode.STATION_INTAKE);
  }

  /**
   * @return Command to set claw to station intake position
   */
  public Command algaeIntake() {
    return setVoltageCommand(ClawMode.ALGAE_INTAKE);
  }

  /**
   * @return Command to set claw to l1 scoring position
   */
  public Command outtake() {
    return setVoltageCommand(ClawMode.OUTTAKE);
  }

  /**
   * @return Command to stop the claw
   */
  public Command stopCommand() {
    return setVoltageCommand(ClawMode.NONE);
  }
}
