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
    FLOOR_INTAKE,
    SOURCE_INTAKE,
    L1,
    L1Back,
    L2,
    L2Back,
    L3,
    L3Back,
    L4,
    L4Back,
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
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.STOW);
        extension.setExtensionPosition(Extension.ExtensionPosition.STOW);
        wrist.setWristPosition(Wrist.WristPosition.STOW);
      }
      case FLOOR_INTAKE -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.FLOOR_INTAKE);
        extension.setExtensionPosition(Extension.ExtensionPosition.FLOOR_INTAKE);
        wrist.setWristPosition(Wrist.WristPosition.FLOOR_INTAKE);
      }
      case SOURCE_INTAKE -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.SOURCE_INTAKE);
        extension.setExtensionPosition(Extension.ExtensionPosition.SOURCE_INTAKE);
        wrist.setWristPosition(Wrist.WristPosition.SOURCE_INTAKE);
      }
      case L1 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L1);
        extension.setExtensionPosition(Extension.ExtensionPosition.L1);
        wrist.setWristPosition(Wrist.WristPosition.L1);
      }
      case L1Back -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L1Back);
        extension.setExtensionPosition(Extension.ExtensionPosition.L1Back);
        wrist.setWristPosition(Wrist.WristPosition.L1Back);
      }
      case L2 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L2);
        extension.setExtensionPosition(Extension.ExtensionPosition.L2);
        wrist.setWristPosition(Wrist.WristPosition.L2);
      }
      case L2Back -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L2Back);
        extension.setExtensionPosition(Extension.ExtensionPosition.L2Back);
        wrist.setWristPosition(Wrist.WristPosition.L2Back);
      }
      case L3 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L3);
        extension.setExtensionPosition(Extension.ExtensionPosition.L3);
        wrist.setWristPosition(Wrist.WristPosition.L3);
      }
      case L3Back -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L3Back);
        extension.setExtensionPosition(Extension.ExtensionPosition.L3Back);
        wrist.setWristPosition(Wrist.WristPosition.L3Back);
      }
      case L4 -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L4);
        extension.setExtensionPosition(Extension.ExtensionPosition.L4);
        wrist.setWristPosition(Wrist.WristPosition.L4);
      }
      case L4Back -> {
        shoulder.setShoulderPosition(Shoulder.ShoulderPosition.L4Back);
        extension.setExtensionPosition(Extension.ExtensionPosition.L4Back);
        wrist.setWristPosition(Wrist.WristPosition.L4Back);
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
