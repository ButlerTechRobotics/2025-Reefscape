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
    FLOOR_INTAKE,
    STATION_INTAKE,
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

  public SmartArm(Arm arm, Goal goal) {
    this.arm = arm;
    this.goal = goal;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    switch (goal) {
      case STOW:
        arm.setGoalCommand(Arm.Goal.STOW).schedule();
        break;
      case FLOOR_INTAKE:
        arm.setGoalCommand(Arm.Goal.FLOOR_INTAKE).schedule();
        break;
      case STATION_INTAKE:
        arm.setGoalCommand(Arm.Goal.STATION_INTAKE).schedule();
        break;
      case L1:
        arm.setGoalCommand(Arm.Goal.L1).schedule();
        break;
      case L1Back:
        arm.setGoalCommand(Arm.Goal.L1Back).schedule();
        break;
      case L2:
        arm.setGoalCommand(Arm.Goal.L2).schedule();
        break;
      case L2Back:
        arm.setGoalCommand(Arm.Goal.L2Back).schedule();
        break;
      case L3:
        arm.setGoalCommand(Arm.Goal.L3).schedule();
        break;
      case L3Back:
        arm.setGoalCommand(Arm.Goal.L3Back).schedule();
        break;
      case L4:
        arm.setGoalCommand(Arm.Goal.L4).schedule();
        break;
      case L4Back:
        arm.setGoalCommand(Arm.Goal.L4Back).schedule();
        break;
      case CLIMB:
        arm.setGoalCommand(Arm.Goal.CLIMB).schedule();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.setGoalCommand(Arm.Goal.STOW).schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
