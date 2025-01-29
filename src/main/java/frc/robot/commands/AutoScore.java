// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;

public class AutoScore extends Command {
  private final Claw claw;
  private final Arm arm;
  private final ArmMode armMode;
  private final ClawMode clawMode;
  private boolean isScored;

  public enum ArmMode {
    STOP,
    INTAKE,
    L1,
    L2,
    L3,
    L4
  }

  public enum ClawMode {
    NONE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    OUTTAKE
  }

  public AutoScore(Claw claw, Arm arm, ArmMode armMode, ClawMode clawMode) {
    this.claw = claw;
    this.arm = arm;
    this.armMode = armMode;
    this.clawMode = clawMode;
    this.isScored = false;

    addRequirements(claw, arm);
  }

  @Override
  public void initialize() {
    // Set the arm mode
    switch (armMode) {
      case STOP:
        arm.stopCommand().schedule();
        break;
      case INTAKE:
        arm.intake().schedule();
        break;
      case L1:
        arm.L1().schedule();
        break;
      case L2:
        arm.L2().schedule();
        break;
      case L3:
        arm.L3().schedule();
        break;
      case L4:
        arm.L4().schedule();
        break;
    }

    // Set the claw mode
    switch (clawMode) {
      case NONE:
        claw.stopCommand().schedule();
        break;
      case FLOOR_INTAKE:
        claw.floorIntake().schedule();
        break;
      case STATION_INTAKE:
        claw.stationIntake().schedule();
        break;
      case OUTTAKE:
        // Do not schedule outtake here, wait until arm is in position
        break;
    }
  }

  @Override
  public void execute() {
    // Check if the arm is in the correct position before outtaking
    if (!isScored && clawMode == ClawMode.OUTTAKE && arm.isAtTarget()) {
      claw.outtake().schedule();
      isScored = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Put the arm back to home
    arm.stopCommand().schedule();
    claw.stopCommand().schedule();
  }

  @Override
  public boolean isFinished() {
    return isScored;
  }
}
