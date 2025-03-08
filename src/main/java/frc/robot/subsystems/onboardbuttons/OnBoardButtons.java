// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.onboardbuttons;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class OnBoardButtons extends SubsystemBase {
  private final OnBoardButtonsIO io;
  private final OnBoardButtonsIOInputsAutoLogged onBoardButtonsInputs =
      new OnBoardButtonsIOInputsAutoLogged();

  public OnBoardButtons(OnBoardButtonsIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(onBoardButtonsInputs);
    Logger.processInputs("OnBoardButtons", onBoardButtonsInputs);
    Logger.recordOutput("OnBoardButtons/HomeButtonPressed", onBoardButtonsInputs.homeButtonPressed);
    Logger.recordOutput(
        "OnBoardButtons/BrakeButtonPressed", onBoardButtonsInputs.brakeButtonPressed);
  }

  public boolean getHomeButtonPressed() {
    return onBoardButtonsInputs.homeButtonPressed;
  }

  public boolean getBrakeButtonPressed() {
    return onBoardButtonsInputs.brakeButtonPressed;
  }
}
