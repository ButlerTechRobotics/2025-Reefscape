// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class AutoArm extends Command {
  private final Arm arm;
  private final ArmMode armMode;
  private boolean isScored;
  private Timer timer = new Timer();

  public enum ArmMode {
    STOW,
    FLOOR_INTAKE,
    STATION_INTAKE,
    L1,
    L2,
    L3,
    L4,
    CLIMB
  }

  public AutoArm(Arm arm, ArmMode armMode) {
    this.arm = arm;
    this.armMode = armMode;
    this.isScored = false;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // Set the arm mode
    switch (armMode) {
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
        arm.setGoalCommand(Arm.Goal.L1Back).schedule();
        break;
      case L2:
        arm.setGoalCommand(Arm.Goal.L2Back).schedule();
        break;
      case L3:
        arm.setGoalCommand(Arm.Goal.L3Back).schedule();
        break;
      case L4:
        arm.setGoalCommand(Arm.Goal.L4Back).schedule();
        break;
      case CLIMB:
        arm.setGoalCommand(Arm.Goal.CLIMB).schedule();
        break;
    }
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Check if the arm is in the correct position
    if (!isScored && arm.isAtTarget()) {
      timer.reset();
    }
    // Wait for half a second before setting isScored to true
    if (timer.hasElapsed(0.5)) {
      isScored = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    // Put the arm back to home
    arm.setGoalCommand(Arm.Goal.STOW).schedule();
  }

  @Override
  public boolean isFinished() {
    return isScored;
  }
}
