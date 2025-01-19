package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ArmVisualizer;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a mechanism with both rotation and extension capabilities for game
 * piece manipulation. It supports multiple positions for different game actions and provides
 * closed-loop control for both joint angle and extension distance.
 */
public class Arm extends SubsystemBase {
  // Hardware interfaces and inputs
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;

  // Current arm position mode
  private ArmPosition currentMode = ArmPosition.INTAKE;

  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;

  // Alerts for motor connection status
  private final Alert jointLeaderAlert =
      new Alert("Joint leader motor isn't connected", AlertType.kError);
  private final Alert jointFollowerAlert =
      new Alert("Joint follower motor isn't connected", AlertType.kError);
  private final Alert jointEncoderAlert =
      new Alert("Joint encoder isn't connected", AlertType.kError);
  private final Alert extensionLeaderAlert =
      new Alert("Extension leader motor isn't connected", AlertType.kError);
  private final Alert extensionFollowerAlert =
      new Alert("Extension follower motor isn't connected", AlertType.kError);
  private final Alert extensionEncoderAlert =
      new Alert("Extension encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Arm subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack, Inches.of(0));
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen, Inches.of(0));
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update motor connection status alerts
    jointLeaderAlert.set(!inputs.jointLeaderConnected);
    jointFollowerAlert.set(!inputs.jointFollowerConnected);
    jointEncoderAlert.set(!inputs.jointEncoderConnected);
    extensionLeaderAlert.set(!inputs.extensionLeaderConnected);
    extensionFollowerAlert.set(!inputs.extensionFollowerConnected);
    extensionEncoderAlert.set(!inputs.extensionEncoderConnected);

    measuredVisualizer.update(inputs.jointAngle, inputs.extensionDistance);
    setpointVisualizer.update(targetJointAngle(), targetExtensionDistance());
  }

  /**
   * Runs the arm in closed-loop position mode to the specified angle and extension.
   *
   * @param jointAngle The target joint angle
   * @param extensionDistance The target extension distance
   */
  private void setPosition(Angle jointAngle, Distance extensionDistance) {
    System.out.println(
        "Setting position: jointAngle=" + jointAngle + ", extensionDistance=" + extensionDistance);
    io.setPosition(jointAngle, extensionDistance);
  }

  /** Stops all arm motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current joint angle of the arm.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getJointPosition() {
    return inputs.jointAngle;
  }

  /**
   * Returns the current extension of the arm.
   *
   * @return The current extension distance
   */
  @AutoLogOutput
  public Distance getExtensionPosition() {
    return inputs.extensionDistance;
  }

  /** Enumeration of available arm positions with their corresponding targets. */
  private enum ArmPosition {
    STOP(Radians.of(0), Meters.of(0)), // Stop the arm
    INTAKE(Radians.of(0), Meters.of(0)), // Arm tucked in
    L1(Radians.of(90), Meters.of(12)), // Position for scoring in L1
    L2(Radians.of(135), Meters.of(24)), // Position for scoring in L2
    L3(Radians.of(135), Meters.of(36)), // Position for scoring in L3
    L4(Radians.of(180), Meters.of(48)); // Position for scoring in L4

    private final Angle targetJointAngle;
    private final Distance targetExtensionDistance;
    private final Angle jointTolerance;
    private final Distance extensionTolerance;

    ArmPosition(
        Angle targetJointAngle,
        Distance targetExtensionDistance,
        Angle jointTolerance,
        Distance extensionTolerance) {
      this.targetJointAngle = targetJointAngle;
      this.targetExtensionDistance = targetExtensionDistance;
      this.jointTolerance = jointTolerance;
      this.extensionTolerance = extensionTolerance;
    }

    ArmPosition(Angle targetJointAngle, Distance targetExtensionDistance) {
      this(
          targetJointAngle,
          targetExtensionDistance,
          Degrees.of(2),
          Inches.of(0.5)); // Default tolerances
    }
  }

  /**
   * Gets the current arm position mode.
   *
   * @return The current ArmPosition
   */
  public ArmPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new arm position and schedules the corresponding command.
   *
   * @param position The desired ArmPosition
   */
  private void setArmPosition(ArmPosition position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Arm"),
              ArmPosition.INTAKE,
              createPositionCommand(ArmPosition.INTAKE),
              ArmPosition.L1,
              createPositionCommand(ArmPosition.L1),
              ArmPosition.L2,
              createPositionCommand(ArmPosition.L2),
              ArmPosition.L3,
              createPositionCommand(ArmPosition.L3),
              ArmPosition.L4,
              createPositionCommand(ArmPosition.L4)),
          this::getMode);

  /**
   * Creates a command for a specific arm position that moves both joint and extension.
   *
   * @param position The arm position to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ArmPosition position) {
    System.out.println(
        "Creating position command: "
            + position.toString()
            + " with jointAngle="
            + position.targetJointAngle
            + " and extensionDistance="
            + position.targetExtensionDistance);
    return Commands.runOnce(
            () -> setPosition(position.targetJointAngle, position.targetExtensionDistance))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ArmPosition.STOP) return true;
    return getJointPosition().isNear(currentMode.targetJointAngle, currentMode.jointTolerance)
        && getExtensionPosition()
            .isNear(currentMode.targetExtensionDistance, currentMode.extensionTolerance);
  }

  /**
   * Gets the target joint angle for the current mode.
   *
   * @return The target joint angle
   */
  @AutoLogOutput
  private Angle targetJointAngle() {
    return currentMode.targetJointAngle;
  }

  /**
   * Gets the target extension distance for the current mode.
   *
   * @return The target extension distance
   */
  @AutoLogOutput
  private Distance targetExtensionDistance() {
    return currentMode.targetExtensionDistance;
  }

  /**
   * Creates a command to set the arm to a specific position.
   *
   * @param position The desired arm position
   * @return Command to set the position
   */
  private Command setPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> setArmPosition(position))
        .withName("SetArmPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to move the arm to L1 scoring position
   */
  public Command L1() {
    return setPositionCommand(ArmPosition.L1);
  }

  /**
   * @return Command to move the arm to L2 scoring position
   */
  public Command L2() {
    return setPositionCommand(ArmPosition.L2);
  }

  /**
   * @return Command to move the arm to L3 position
   */
  public Command L3() {
    return setPositionCommand(ArmPosition.L3);
  }

  /**
   * @return Command to move the arm to L4 position
   */
  public Command L4() {
    return setPositionCommand(ArmPosition.L4);
  }

  /**
   * @return Command to move to intake position
   */
  public Command intake() {
    return setPositionCommand(ArmPosition.INTAKE);
  }

  /**
   * @return Command to stop the arm
   */
  public Command stopCommand() {
    return setPositionCommand(ArmPosition.STOP);
  }
}
