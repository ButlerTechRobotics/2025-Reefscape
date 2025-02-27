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
        // wrist.stow();
        // if (wrist.isAtTarget()) {
        //   extension.stow();
        // }
        // if (shoulder.isAtTarget()) {
        //   shoulder.stow();
        // }
        shoulder.stow();
      }
      case CORAL_FLOOR_INTAKE -> {
        shoulder.coralFloorIntake();
        extension.coralFloorIntake();
        wrist.coralFloorIntake();
      }
      case CORAL_STATION_INTAKE -> {
        shoulder.coralStationIntake();
        extension.coralStationIntake();
        wrist.coralStationIntake();
      }
      case CORAL_L1 -> {
        shoulder.coralL1();
        extension.coralL1();
        wrist.coralL1();
      }
      case CORAL_L1BACK -> {
        shoulder.coralL1Back();
        extension.coralL1Back();
        wrist.coralL1Back();
      }
      case CORAL_L2 -> {
        shoulder.coralL2();
        extension.coralL2();
        wrist.coralL2();
      }
      case CORAL_L2BACK -> {
        shoulder.coralL2Back();
        extension.coralL2Back();
        wrist.coralL2Back();
      }
      case CORAL_L3 -> {
        shoulder.coralL3();
        extension.coralL3();
        wrist.coralL3();
      }
      case CORAL_L3BACK -> {
        shoulder.coralL3Back();
        extension.coralL3Back();
        wrist.coralL3Back();
      }
      case CORAL_L4BACK -> {
        shoulder.coralL4Back();
        extension.coralL4Back();
        wrist.coralL4Back();
      }
      case ALGAE_FLOOR_INTAKE -> {
        shoulder.algaeFloorIntake();
        extension.algaeFloorIntake();
        wrist.algaeFloorIntake();
      }
      case ALGAE_SCORE -> {
        shoulder.algaeScore();
        extension.algaeScore();
        wrist.algaeScore();
      }
      case ALGAE_L1 -> {
        shoulder.algaeL1();
        extension.algaeL1();
        wrist.algaeL1();
      }
      case ALGAE_L2 -> {
        shoulder.algaeL2();
        extension.algaeL2();
        wrist.algaeL2();
      }
      case CLIMB -> {
        shoulder.climb();
        extension.climb();
        wrist.climb();
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
