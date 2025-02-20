// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.extension.Extension;
import frc.robot.subsystems.arm.shoulder.Shoulder;
import frc.robot.subsystems.arm.wrist.Wrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public enum Goal {
    STOW,
    CORAL_FLOOR_INTAKE,
    CORAL_STATION_INTAKE,
    CORAL_L1,
    CORAL_L1BACK,
    CORAL_L2,
    CORAL_L2BACK,
    CORAL_L3,
    CORAL_L3BACK,
    CORAL_L4BACK,
    ALGAE_FLOOR_INTAKE,
    ALGAE_SCORE,
    ALGAE_L1,
    ALGAE_L2,
    CLIMB
  }

  private Goal currentGoal = Goal.STOW;
  private Goal desiredGoal = Goal.STOW;

  private final Shoulder shoulder;
  private final Extension extension;
  private final Wrist wrist;

  private Timer goalTimer = new Timer();

  private final ArmVisualizer measuredVisualizer = new ArmVisualizer("Measured");
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("Setpoint");

  public Arm(Shoulder shoulder, Extension extension, Wrist wrist) {
    this.shoulder = shoulder;
    this.extension = extension;
    this.wrist = wrist;

    setDefaultCommand(setGoalCommand(Goal.STOW));
    goalTimer.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
      shoulder.stopCommand();
      extension.stopCommand();
      wrist.stopCommand();
    }

    // Update current goal to desired goal
    if (currentGoal != desiredGoal) {
      currentGoal = desiredGoal;
      goalTimer.reset();
    }

    switch (currentGoal) {
      case STOW -> {
        wrist.setWristPosition(Wrist.WristPosition.STOW);
        if (wrist.isAtTarget()) {
          extension.setExtensionPosition(Extension.ExtensionPosition.STOW);
        }
        if (shoulder.isAtTarget()) {
          shoulder.setShoulderPosition(Shoulder.ShoulderPosition.STOW);
        }
      }
      case CORAL_FLOOR_INTAKE -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_FLOOR_INTAKE);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_FLOOR_INTAKE);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_FLOOR_INTAKE);
      }
      case CORAL_STATION_INTAKE -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_STATION_INTAKE);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_STATION_INTAKE);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_STATION_INTAKE);
      }
      case CORAL_L1 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L1);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L1);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L1);
      }
      case CORAL_L1BACK -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L1BACK);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L1BACK);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L1BACK);
      }
      case CORAL_L2 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L2);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L2);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L2);
      }
      case CORAL_L2BACK -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L2BACK);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L2BACK);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L2BACK);
      }
      case CORAL_L3 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L3);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L3);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L3);
      }
      case CORAL_L3BACK -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L3BACK);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L3BACK);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L3BACK);
      }
      case CORAL_L4BACK -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CORAL_L4BACK);
        extension.setExtensionPosition(Extension.ExtensionPosition.CORAL_L4BACK);
        wrist.setWristPosition(Wrist.WristPosition.CORAL_L4BACK);
      }
      case ALGAE_FLOOR_INTAKE -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.ALGAE_FLOOR_INTAKE);
        extension.setExtensionPosition(Extension.ExtensionPosition.ALGAE_FLOOR_INTAKE);
        wrist.setWristPosition(Wrist.WristPosition.ALGAE_FLOOR_INTAKE);
      }
      case ALGAE_SCORE -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.ALGAE_SCORE);
        extension.setExtensionPosition(Extension.ExtensionPosition.ALGAE_SCORE);
        wrist.setWristPosition(Wrist.WristPosition.ALGAE_SCORE);
      }
      case ALGAE_L1 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.ALGAE_L1);
        extension.setExtensionPosition(Extension.ExtensionPosition.ALGAE_L1);
        wrist.setWristPosition(Wrist.WristPosition.ALGAE_L1);
      }
      case ALGAE_L2 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.ALGAE_L2);
        extension.setExtensionPosition(Extension.ExtensionPosition.ALGAE_L2);
        wrist.setWristPosition(Wrist.WristPosition.ALGAE_L2);
      }
      case CLIMB -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.CLIMB);
        extension.setExtensionPosition(Extension.ExtensionPosition.CLIMB);
        wrist.setWristPosition(Wrist.WristPosition.CLIMB);
      }
    }
    shoulder.periodic();
    extension.periodic();
    wrist.periodic();

    Logger.recordOutput("Arm/GoalState", desiredGoal);
    Logger.recordOutput("Arm/CurrentState", currentGoal);

    // Update visualizer
    measuredVisualizer.update(shoulder.getPosition(), extension.getPosition(), wrist.getPosition());
    setpointVisualizer.update(
        shoulder.targetAngle(), extension.targetDistance(), wrist.targetAngle());
  }

  public Shoulder getShoulder() {
    return shoulder;
  }

  public Extension getExtension() {
    return extension;
  }

  public Wrist getWrist() {
    return wrist;
  }

  /** Set goal of arm */
  private void setGoal(Goal goal) {
    if (desiredGoal == goal) return;
    desiredGoal = goal;
  }

  /** Command to set goal of arm */
  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW)).withName("Arm " + goal);
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  public boolean isAtTarget() {
    return currentGoal == desiredGoal
        && shoulder.isAtTarget()
        && extension.isAtTarget()
        && wrist.isAtTarget();
  }

  @AutoLogOutput(key = "Arm/AtShoulderGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && shoulder.isAtTarget();
  }

  @AutoLogOutput(key = "Arm/AtExtensionGoal")
  public boolean atExtensionGoal() {
    return currentGoal == desiredGoal && extension.isAtTarget();
  }

  @AutoLogOutput(key = "Arm/AtWristGoal")
  public boolean atWristGoal() {
    return currentGoal == desiredGoal && wrist.isAtTarget();
  }
}
