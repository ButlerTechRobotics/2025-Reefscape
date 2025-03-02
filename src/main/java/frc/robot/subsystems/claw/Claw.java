// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Simple claw subsystem for game piece manipulation. */
public class Claw extends SubsystemBase {
  // Hardware and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  private final Alert motorAlert = new Alert("Claw motor disconnected", Alert.AlertType.kError);

  // Constants for voltages
  private static final Voltage STOP_VOLTAGE = Volts.of(0);
  private static final Voltage INTAKE_VOLTAGE = Volts.of(12);
  private static final Voltage OUTTAKE_VOLTAGE = Volts.of(-12);

  // Track current state
  private String currentState = "STOPPED";

  public Claw(ClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs and alerts
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    motorAlert.set(!inputs.leaderConnected);
  }

  /** Sets the claw voltage directly */
  private void setVoltage(Voltage voltage, String stateName) {
    if (voltage.equals(STOP_VOLTAGE)) {
      io.stop();
    } else {
      io.setVoltage(voltage);
    }
    currentState = stateName;
    Logger.recordOutput("Claw/State", currentState);
    Logger.recordOutput("Claw/Voltage", voltage.in(Volts));
  }

  /**
   * @return Current claw position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.leaderPosition;
  }

  /**
   * @return Current claw state
   */
  @AutoLogOutput(key = "Claw/CurrentState")
  public String getState() {
    return currentState;
  }

  // Command factory methods - simpler implementations with clear names

  /**
   * @return Command to run intake at floor
   */
  public Command intake() {
    return Commands.runOnce(() -> setVoltage(INTAKE_VOLTAGE, "INTAKE"), this);
  }

  /**
   * @return Command to outtake/eject game piece
   */
  public Command outtake() {
    return Commands.runOnce(() -> setVoltage(OUTTAKE_VOLTAGE, "OUTTAKE"), this);
  }

  /**
   * @return Command to stop the claw
   */
  public Command stop() {
    return Commands.runOnce(() -> setVoltage(STOP_VOLTAGE, "STOPPED"), this);
  }
}
