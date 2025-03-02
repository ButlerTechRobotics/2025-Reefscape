// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class SmartClaw extends Command {
  private final Claw claw;
  private final Goal goal;

  public enum Goal {
    STOW,
    CORAL_FLOOR_INTAKE,
    CORAL_STATION_INTAKE,
    ALGAE_INTAKE,
    OUTTAKE
  }

  public SmartClaw(Claw claw, Goal goal) {
    this.claw = claw;
    this.goal = goal;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    switch (goal) {
      case STOW:
        claw.stop().schedule();
        break;
      case CORAL_FLOOR_INTAKE:
        claw.intake().schedule();
        break;
      case CORAL_STATION_INTAKE:
        claw.intake().schedule();
        break;
      case ALGAE_INTAKE:
        claw.intake().schedule();
        break;
      case OUTTAKE:
        claw.outtake().schedule();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    claw.stop().schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
