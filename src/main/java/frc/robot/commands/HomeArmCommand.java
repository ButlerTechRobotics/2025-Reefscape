// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;

/** Zeros the arm. Sets the proper 0 values for the shoulder, extension, and wrist. */
public class HomeArmCommand extends Command {
  // Velocity thresholds to detect when mechanisms hit limits
  private static final double WRIST_VELOCITY_THRESHOLD = Units.degreesToRadians(0.1);
  private static final double EXTENSION_VELOCITY_THRESHOLD = Units.inchesToMeters(0.1);
  private static final double SHOULDER_VELOCITY_THRESHOLD = Units.degreesToRadians(0.1);

  // Zeroing voltages (adjust as needed for your mechanisms)
  private static final Voltage WRIST_ZEROING_VOLTAGE = Volts.of(1.5);
  private static final Voltage EXTENSION_ZEROING_VOLTAGE = Volts.of(-1.0);
  private static final Voltage SHOULDER_ZEROING_VOLTAGE = Volts.of(-1.0);

  // Time required to confirm zeroing
  private static final double ZERO_VELOCITY_TIME_PERIOD = 0.5;

  private final Arm arm;
  private final Timer timer = new Timer();
  private double zeroTimeStamp;

  // Enum to track current zeroing step
  private enum HomingState {
    ZEROING_WRIST,
    ZEROING_EXTENSION,
    ZEROING_SHOULDER,
    FINISHED
  }

  private HomingState currentState = HomingState.ZEROING_WRIST;

  public HomeArmCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm, arm.getShoulder(), arm.getExtension(), arm.getWrist());
  }

  @Override
  public void initialize() {
    currentState = HomingState.ZEROING_WRIST;
    zeroTimeStamp = Double.NaN;
    arm.setZeroed(false);
    timer.reset();
    timer.start();
    Logger.recordOutput("Arm/HomingState", currentState.toString());
  }

  @Override
  public void execute() {
    switch (currentState) {
      case ZEROING_WRIST:
        // Check if wrist has stopped moving
        if (manageTimer(getWristVelocity(), WRIST_VELOCITY_THRESHOLD)) {
          // Wrist is at limit, zero sensor
          // arm.getWrist().setZero();
          arm.getWrist().stopCommand();
          currentState = HomingState.ZEROING_EXTENSION;
          zeroTimeStamp = Double.NaN;
          Logger.recordOutput("Arm/HomingState", currentState.toString());
        } else {
          // Apply voltage to move wrist to limit
          arm.getWrist().setVoltage(WRIST_ZEROING_VOLTAGE);
        }
        break;

      case ZEROING_EXTENSION:
        // Check if extension has stopped moving
        if (manageTimer(getExtensionVelocity(), EXTENSION_VELOCITY_THRESHOLD)) {
          // Extension is at limit, zero sensor
          // arm.getExtension().setZero();
          arm.getExtension().stopCommand();
          currentState = HomingState.ZEROING_SHOULDER;
          zeroTimeStamp = Double.NaN;
          Logger.recordOutput("Arm/HomingState", currentState.toString());
        } else {
          // Apply voltage to move extension to limit
          arm.getExtension().setVoltage(EXTENSION_ZEROING_VOLTAGE);
        }
        break;

      case ZEROING_SHOULDER:
        // Check if shoulder has stopped moving
        if (manageTimer(
            getShoulderVelocity().abs(RotationsPerSecond), SHOULDER_VELOCITY_THRESHOLD)) {
          // Shoulder is at limit, zero sensor
          arm.getShoulder().setZero();
          arm.getShoulder().stopCommand();
          arm.setZeroed(true);
          currentState = HomingState.FINISHED;
          Logger.recordOutput("Arm/HomingState", currentState.toString());
        } else {
          // Apply voltage to move shoulder to limit
          arm.getShoulder().setVoltage(SHOULDER_ZEROING_VOLTAGE);
        }
        break;

      case FINISHED:
        // Nothing to do here
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    arm.getWrist().stopCommand();
    arm.getExtension().stopCommand();
    arm.getShoulder().stopCommand();

    if (!interrupted) {
      arm.setZeroed(true);
      arm.setGoalCommand(Arm.Goal.STOW).schedule();
    }

    Logger.recordOutput("Arm/HomingCompleted", !interrupted);
  }

  @Override
  public boolean isFinished() {
    return arm.isZeroed();
  }

  private boolean manageTimer(double velocity, double threshold) {
    if (Math.abs(velocity) < threshold) {
      if (!Double.isFinite(zeroTimeStamp)) {
        zeroTimeStamp = Timer.getFPGATimestamp();
        return false;
      } else {
        return Timer.getFPGATimestamp() - zeroTimeStamp >= ZERO_VELOCITY_TIME_PERIOD;
      }
    } else {
      zeroTimeStamp = Double.NaN;
      return false;
    }
  }

  // Helper methods to get velocities - add these to your subsystem interfaces if needed
  private double getWristVelocity() {
    // You'll need to implement this based on your wrist subsystem
    return 0.0; // Placeholder
  }

  private double getExtensionVelocity() {
    // You'll need to implement this based on your extension subsystem
    return 0.0; // Placeholder
  }

  private AngularVelocity getShoulderVelocity() {
    // You'll need to implement this based on your shoulder subsystem
    return arm.getShoulder().getVelocity();
  }
}
