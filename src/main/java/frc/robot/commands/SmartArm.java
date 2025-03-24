// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class SmartArm extends Command {
  private final Arm arm;
  private final Goal goal;

  public enum Goal {
    STOW,
    STANDBY,
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

  public SmartArm(Arm arm, Goal goal) {
    this.arm = arm;
    this.goal = goal;
    addRequirements(arm, arm.getShoulder(), arm.getExtension(), arm.getWrist());
  }

  @Override
  public void initialize() {
    switch (goal) {
      case STOW:
        arm.setGoalCommand(Arm.Goal.STOW).schedule();
        break;
      case STANDBY:
        arm.setGoalCommand(Arm.Goal.STANDBY).schedule();
        break;
      case CORAL_FLOOR_INTAKE:
        arm.setGoalCommand(Arm.Goal.CORAL_FLOOR_INTAKE).schedule();
        break;
      case CORAL_STATION_INTAKE:
        arm.setGoalCommand(Arm.Goal.CORAL_STATION_INTAKE).schedule();
        break;
      case CORAL_L1:
        arm.setGoalCommand(Arm.Goal.CORAL_L1).schedule();
        break;
      case CORAL_L1BACK:
        arm.setGoalCommand(Arm.Goal.CORAL_L1BACK).schedule();
        break;
      case CORAL_L2:
        arm.setGoalCommand(Arm.Goal.CORAL_L2).schedule();
        break;
      case CORAL_L2BACK:
        arm.setGoalCommand(Arm.Goal.CORAL_L2BACK).schedule();
        break;
      case CORAL_L3:
        arm.setGoalCommand(Arm.Goal.CORAL_L3).schedule();
        break;
      case CORAL_L3BACK:
        arm.setGoalCommand(Arm.Goal.CORAL_L3BACK).schedule();
        break;
      case CORAL_L4BACK:
        arm.setGoalCommand(Arm.Goal.CORAL_L4BACK).schedule();
        break;
      case ALGAE_FLOOR_INTAKE:
        arm.setGoalCommand(Arm.Goal.ALGAE_FLOOR_INTAKE).schedule();
        break;
      case ALGAE_SCORE:
        arm.setGoalCommand(Arm.Goal.ALGAE_SCORE).schedule();
        break;
      case ALGAE_L1:
        arm.setGoalCommand(Arm.Goal.ALGAE_L1).schedule();
        break;
      case ALGAE_L2:
        arm.setGoalCommand(Arm.Goal.ALGAE_L2).schedule();
        break;
      case CLIMB:
        arm.setGoalCommand(Arm.Goal.CLIMB).schedule();
        break;
      case CLIMB_DOWN:
        arm.setGoalCommand(Arm.Goal.CLIMB_DOWN).schedule();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.setGoalCommand(Arm.Goal.STANDBY).schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
