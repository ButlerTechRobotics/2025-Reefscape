// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DisabledInstantCommand extends InstantCommand {
  public DisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
