package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
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
 * The Claw subsystem controls a motor-driven claw mechanism for game piece manipulation. It
 * supports multiple speeds for different game actions and provides both open-loop and closed-loop
 * control options.
 */
public class Claw extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs;

  // Alerts for motor connection status
  private final Alert motorAlert = new Alert("Claw motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Claw encoder isn't connected", AlertType.kError);

  // System identification routine configuration
  private final SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduced voltage for claw to prevent damage
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              state -> Logger.recordOutput("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                this.runVoltage(output);
                Logger.recordOutput("Claw_Position", output.in(Volts));
              },
              null,
              this));

  // Current claw position mode
  private ClawMode currentMode = ClawMode.NONE;

  /**
   * Creates a new Claw subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the claw
   */
  public Claw(ClawIO io) {
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
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the claw in open-loop mode at the specified voltage.
   *
   * @param volts The voltage to apply to the motor
   */
  public void runVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  /** Stops the claw motor. */
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
   * Returns the current position of the claw.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /** Enumeration of available claw positions with their corresponding target angles. */
  public enum ClawMode {
    NONE(Voltage.ofBaseUnits(0, Volts)), // Stowed position
    FLOOR_INTAKE(Voltage.ofBaseUnits(3, Volts)), // Position for floor intake
    STATION_INTAKE(Voltage.ofBaseUnits(2, Volts)), // Position for station intake
    OUTTAKE(Voltage.ofBaseUnits(6, Volts)); // Position for gripping at level 1

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
              Commands.runOnce(this::stop)
                  .withName("Stop Claw"),
              ClawMode.FLOOR_INTAKE,
              createPositionCommand(ClawMode.FLOOR_INTAKE),
              ClawMode.STATION_INTAKE,
              createPositionCommand(ClawMode.STATION_INTAKE),
              ClawMode.OUTTAKE,
              createPositionCommand(ClawMode.OUTTAKE)),
          this::getMode);

  /**
   * Creates a command for a specific claw voltage that spins the claw and checks the target
   * voltage.
   *
   * @param voltage The claw voltage to create a command for
   * @return A command that implements the claw movement
   */
  private Command createPositionCommand(ClawMode voltage) {
    return Commands.parallel(
        Commands.runOnce(() -> runVoltage(voltage.voltage))
            .withName("Move to " + voltage.toString()));
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
