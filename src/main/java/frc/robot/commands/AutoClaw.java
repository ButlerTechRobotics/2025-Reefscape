// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class AutoClaw extends Command {
  private final Claw claw;
  private final ClawMode clawMode;
  private boolean isScored;
  private Timer timer = new Timer();

  public enum ClawMode {
    NONE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    OUTTAKE
  }

  public AutoClaw(Claw claw, ClawMode clawMode) {
    this.claw = claw;
    this.clawMode = clawMode;
    this.isScored = false;

    addRequirements(claw);
  }

  public void initialize() {
    // Set the claw mode
    switch (clawMode) {
      case NONE:
        claw.setVoltageCommand(Claw.ClawMode.NONE).schedule();
        break;
      case FLOOR_INTAKE:
        claw.setVoltageCommand(Claw.ClawMode.FLOOR_INTAKE).schedule();
        break;
      case STATION_INTAKE:
        claw.setVoltageCommand(Claw.ClawMode.STATION_INTAKE).schedule();
        break;
      case OUTTAKE:
        claw.setVoltageCommand(Claw.ClawMode.OUTTAKE).schedule();
        break;
    }
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Check if the arm is in the correct position
    if (!isScored) {
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
    claw.setVoltageCommand(Claw.ClawMode.NONE).schedule();
  }

  @Override
  public boolean isFinished() {
    return isScored;
  }
}
