// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.onboardbuttons;

import org.littletonrobotics.junction.Logger;

public class OnBoardButtons {
  private final OnBoardButtonsIO io;
  private final OnBoardButtonsIOInputsAutoLogged onBoardButtonsInputs =
      new OnBoardButtonsIOInputsAutoLogged();

  public OnBoardButtons(OnBoardButtonsIO io) {
    this.io = io;
  }

  public void update() {
    io.updateInputs(onBoardButtonsInputs);
    Logger.processInputs("OnBoardButtons", onBoardButtonsInputs);
  }

  public boolean getHomeButtonPressed() {
    return onBoardButtonsInputs.homeButtonPressed;
  }

  public boolean getBrakeButtonPressed() {
    return onBoardButtonsInputs.brakeButtonPressed;
  }
}
