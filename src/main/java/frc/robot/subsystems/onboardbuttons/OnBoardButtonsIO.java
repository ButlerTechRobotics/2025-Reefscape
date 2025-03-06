// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.onboardbuttons;

import org.littletonrobotics.junction.AutoLog;

public interface OnBoardButtonsIO {
  default void updateInputs(OnBoardButtonsIOInputs inputs) {}

  @AutoLog
  class OnBoardButtonsIOInputs {
    public boolean homeButtonPressed;
    public boolean brakeButtonPressed;
  }
}
