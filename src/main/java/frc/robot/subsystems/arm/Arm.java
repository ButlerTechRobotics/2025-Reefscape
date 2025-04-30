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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.extension.Extension;
import frc.robot.subsystems.arm.shoulder.Shoulder;
import frc.robot.subsystems.arm.wrist.Wrist;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public enum Goal {
    STOW,
    STANDBY,
    CORAL_PRE_INTAKE,
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
    CLIMB,
    CLIMB_DOWN
  }

  private Goal currentGoal = Goal.STOW;
  private Goal desiredGoal = Goal.STOW;

  private final Shoulder shoulder;
  private final Extension extension;
  private final Wrist wrist;
  private final Intake intake;

  private Timer goalTimer = new Timer();

  private final ArmVisualizer measuredVisualizer = new ArmVisualizer("Measured");
  private final ArmVisualizer setpointVisualizer = new ArmVisualizer("Setpoint");

  private boolean zeroed = false;

  public Arm(Shoulder shoulder, Extension extension, Wrist wrist, Intake intake) {
    this.shoulder = shoulder;
    this.extension = extension;
    this.wrist = wrist;
    this.intake = intake;

    setDefaultCommand(setGoalCommand(Goal.STOW));
    goalTimer.start();
  }

  @Override
  public void periodic() {
    shoulder.setIsExtended(extension.isExtended());
    extension.setIsVertical(shoulder.isVertical());
    wrist.setHasGamePiece(intake.hasGamePiece());

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
        wrist.stow();
        if (wrist.isAtTarget()) {
          shoulder.stow();
        }
      }
      case STANDBY -> {
        shoulder.standby().schedule();
        if (shoulder.isAtTarget()) {
          extension.standby().schedule();
          wrist.standby().schedule();
        }
      }
      case CORAL_PRE_INTAKE -> {
        shoulder.coralPreIntake().schedule();
        wrist.coralPreIntake().schedule();
        extension.coralPreIntake().schedule();
      }
      case CORAL_FLOOR_INTAKE -> {
        extension.coralFloorIntake().schedule();
        wrist.coralFloorIntake().schedule();
        shoulder.coralFloorIntake().schedule();
      }
      case CORAL_STATION_INTAKE -> {
        shoulder.coralStationIntake().schedule();
        if (shoulder.isAtTarget()) {
          wrist.coralStationIntake().schedule();
        }
        if (wrist.isAtTarget()) {
          extension.coralStationIntake().schedule();
        }
      }
      case CORAL_L1 -> {
        shoulder.coralL1().schedule();
        extension.coralL1().schedule();
        wrist.coralL1().schedule();
      }
      case CORAL_L1BACK -> {
        shoulder.coralL1Back().schedule();
        extension.coralL1Back().schedule();
        wrist.coralL1Back().schedule();
      }
      case CORAL_L2 -> {
        shoulder.coralL2().schedule();
        extension.coralL2().schedule();
        wrist.coralL2().schedule();
      }
      case CORAL_L2BACK -> {
        shoulder.coralL2Back().schedule();
        extension.coralL2Back().schedule();
        wrist.coralL2Back().schedule();
      }
      case CORAL_L3 -> {
        shoulder.coralL3().schedule();
        extension.coralL3().schedule();
        wrist.coralL3().schedule();
      }
      case CORAL_L3BACK -> {
        wrist.coralL3Back().schedule();
        extension.coralL3Back().schedule();
        shoulder.coralL3Back().schedule();
      }
      case CORAL_L4BACK -> {
        wrist.coralL4Back().schedule();
        shoulder.coralL4Back().schedule();
        extension.coralL4Back().schedule();
      }
      case ALGAE_FLOOR_INTAKE -> {
        shoulder.algaeFloorIntake().schedule();
        extension.algaeFloorIntake().schedule();
        wrist.algaeFloorIntake().schedule();
      }
      case ALGAE_SCORE -> {
        shoulder.algaeScore().schedule();
        extension.algaeScore().schedule();
        wrist.algaeScore().schedule();
      }
      case ALGAE_L1 -> {
        shoulder.algaeL1().schedule();
        extension.algaeL1().schedule();
        wrist.algaeL1().schedule();
      }
      case ALGAE_L2 -> {
        shoulder.algaeL2().schedule();
        extension.algaeL2().schedule();
        wrist.algaeL2().schedule();
      }
      case CLIMB -> {
        shoulder.climb().schedule();
        extension.climb().schedule();
        wrist.climb().schedule();
      }
      case CLIMB_DOWN -> {
        shoulder.climbDown().schedule();
        extension.climbDown().schedule();
        wrist.climbDown().schedule();
      }
    }

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

  public Goal getGoal() {
    return currentGoal;
  }

  /** Command to set goal of arm */
  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STANDBY)).withName("Arm " + goal);
  }

  /** Command to set goal of arm */
  public Command setGoalAutoCommand(Goal goal) {
    return runOnce(() -> setGoal(goal)).withName("Arm " + goal);
  }

  /**
   * Safely go to CORAL_PRE_INTAKE based on current position. Only routes through STANDBY if
   * currently in CORAL_L4BACK position.
   *
   * @return A command sequence that takes the safest path to pre-intake
   */
  public Command safeGoToPreIntake() {
    return Commands.either(
            // If in L4BACK, go through STANDBY first
            Commands.sequence(
                // First go to STANDBY position
                setGoalAutoCommand(Goal.STANDBY),
                // Wait until arm is at STANDBY position
                Commands.waitUntil(this::isAtTarget),
                // Then go to PRE_INTAKE position
                setGoalAutoCommand(Goal.CORAL_PRE_INTAKE)),
            // Else, skip STANDBY and go through PRE_INTAKE only
            Commands.sequence(
                // Go to PRE_INTAKE position
                setGoalAutoCommand(Goal.CORAL_PRE_INTAKE),
                // Wait until arm is at PRE_INTAKE position
                Commands.waitUntil(this::isAtTarget)),
            // Condition: if currentGoal is CORAL_L4BACK
            () -> currentGoal == Goal.CORAL_L4BACK)
        .withName("Safe Pre-Intake");
  }

  /**
   * Safely go to CORAL_FLOOR_INTAKE based on current position. Routes through STANDBY only if
   * currently in CORAL_L4BACK position.
   *
   * @return A command sequence that takes the safest path to floor intake
   */
  public Command safeGoToFloorIntake() {
    return Commands.either(
            // If in L4BACK, go through STANDBY first
            Commands.sequence(
                // First go to STANDBY position
                setGoalAutoCommand(Goal.STANDBY),
                // Wait until arm is at STANDBY position
                Commands.waitUntil(this::isAtTarget),
                // Then go to PRE_INTAKE position
                setGoalAutoCommand(Goal.CORAL_PRE_INTAKE),
                // Wait until arm is at PRE_INTAKE position
                Commands.waitUntil(this::isAtTarget),
                // Finally go to FLOOR_INTAKE position
                setGoalAutoCommand(Goal.CORAL_FLOOR_INTAKE)),
            // Else, skip STANDBY and go through PRE_INTAKE only
            Commands.sequence(
                // Go to PRE_INTAKE position
                setGoalAutoCommand(Goal.CORAL_PRE_INTAKE),
                // Wait until arm is at PRE_INTAKE position
                Commands.waitUntil(this::isAtTarget),
                // Then go to FLOOR_INTAKE position
                setGoalAutoCommand(Goal.CORAL_FLOOR_INTAKE)),
            // Condition: if currentGoal is CORAL_L4BACK
            () -> currentGoal == Goal.CORAL_L4BACK)
        .withName("Safe Floor Intake");
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

  public void setZeroed(boolean zero) {
    this.zeroed = zero;
  }

  @AutoLogOutput(key = "Arm/IsZeroed")
  public boolean isZeroed() {
    return zeroed;
  }

  @AutoLogOutput(key = "Arm/IsScoringFront")
  public boolean isScoringFront() {
    return currentGoal == Goal.CORAL_L2 || currentGoal == Goal.CORAL_L3;
  }
}
