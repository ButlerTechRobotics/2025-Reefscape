package frc.robot.subsystems.claw_joint;

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
 * The ClawJoint subsystem controls a motor-driven claw mechanism for game piece manipulation. It
 * supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class ClawJoint extends SubsystemBase {
  // Hardware interface and inputs
  private final ClawJointIO io;
  private final ClawJointIOInputsAutoLogged inputs;

  // Alerts for motor connection status
  private final Alert motorAlert = new Alert("Claw-Joint motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Claw-Joint encoder isn't connected", AlertType.kError);

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
                Logger.recordOutput("Claw-Joint_Position", output.in(Volts));
              },
              null,
              this));

  // Current claw position mode
  private ClawPosition currentMode = ClawPosition.STOWED;

  /**
   * Creates a new Claw subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the claw
   */
  public ClawJoint(ClawJointIO io) {
    this.io = io;
    this.inputs = new ClawJointIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Claw-Joint", inputs);

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

  /**
   * Runs the claw in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  public void setPosition(Angle position) {
    io.setPosition(position);
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
  public enum ClawPosition {
    STOWED(Degrees.of(0)), // Stowed position
    FLOOR_INTAKE(Degrees.of(0)), // Position for floor intake
    STATION_INTAKE(Degrees.of(45)), // Position for station intake
    L1(Degrees.of(30)), // Position for gripping at level 1
    L2(Degrees.of(15)), // Position for gripping at level 2
    L3(Degrees.of(15)), // Position for gripping at level 3
    L4(Degrees.of(15)); // Position for gripping at level 4


    private final Angle targetAngle;
    private final Angle angleTolerance;

    ClawPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ClawPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(1)); // 1 degree default tolerance for claw
    }
  }

  /**
   * Gets the current claw position mode.
   *
   * @return The current ClawPosition
   */
  public ClawPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new claw position and schedules the corresponding command.
   *
   * @param position The desired ClawPosition
   */
  public void setClawPosition(ClawPosition position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ClawPosition.STOWED,
              Commands.runOnce(this::stop)
                  .alongWith(Commands.run(() -> checkAtTarget(ClawPosition.STOWED)))
                  .withName("Stow Claw"),
              ClawPosition.FLOOR_INTAKE,
              createPositionCommand(ClawPosition.FLOOR_INTAKE),
              ClawPosition.STATION_INTAKE,
              createPositionCommand(ClawPosition.STATION_INTAKE),
              ClawPosition.L1,
              createPositionCommand(ClawPosition.L1),
              ClawPosition.L2,
              createPositionCommand(ClawPosition.L2),
              ClawPosition.L3,
              createPositionCommand(ClawPosition.L3),
              ClawPosition.L4,
              createPositionCommand(ClawPosition.L4)),
          this::getMode);

  /**
   * Creates a command for a specific claw position that moves the claw and checks the target
   * position.
   *
   * @param position The claw position to create a command for
   * @return A command that implements the claw movement
   */
  private Command createPositionCommand(ClawPosition position) {
    return Commands.parallel(
        Commands.runOnce(() -> setPosition(position.targetAngle))
            .withName("Move to " + position.toString()),
        Commands.run(() -> checkAtTarget(position))
            .withName("Check " + position.toString() + " Target"));
  }

  /**
   * Checks if the claw is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  private boolean isAtTarget() {
    if (currentMode == ClawPosition.STOWED) return true;
    return getPosition().isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  /**
   * Logs whether the claw is at its target position for a given mode.
   *
   * @param position The position to check against
   */
  private void checkAtTarget(ClawPosition position) {
    boolean atTarget = isAtTarget();
    Logger.recordOutput("Claw/AtTarget", atTarget);
    Logger.recordOutput("Claw/TargetAngle", position.targetAngle);
  }

  /**
   * Creates a command to set the claw to a specific position.
   *
   * @param position The desired claw position
   * @return Command to set the position
   */
  public Command setPositionCommand(ClawPosition position) {
    return Commands.runOnce(() -> setClawPosition(position))
        .withName("SetClawPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to set claw to stowed position
   */
  public Command stow() {
    return setPositionCommand(ClawPosition.STOWED);
  }

  /**
   * @return Command to set claw to floor intake position
   */
  public Command floorIntake() {
    return setPositionCommand(ClawPosition.FLOOR_INTAKE);
  }

  /**
   * @return Command to set claw to station intake position
   */
  public Command stationIntake() {
    return setPositionCommand(ClawPosition.STATION_INTAKE);
  }

  /**
   * @return Command to set claw to l1 scoring position
   */
  public Command l1() {
    return setPositionCommand(ClawPosition.L1);
  }

   /**
   * @return Command to set claw to l2 scoring position
   */
  public Command l2() {
    return setPositionCommand(ClawPosition.L2);
  }

   /**
   * @return Command to set claw to l3 scoring position
   */
  public Command l3() {
    return setPositionCommand(ClawPosition.L3);
  }

   /**
   * @return Command to set claw to l4 scoring position
   */
  public Command l4() {
    return setPositionCommand(ClawPosition.L4);
  }

  /**
   * @return Command to stop the claw
   */
  public Command stopCommand() {
    return setPositionCommand(ClawPosition.STOWED);
  }
}
