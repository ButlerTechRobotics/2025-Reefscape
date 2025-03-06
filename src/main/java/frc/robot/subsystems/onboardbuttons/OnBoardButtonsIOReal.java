// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.onboardbuttons;

import edu.wpi.first.wpilibj.DigitalInput;

public class OnBoardButtonsIOReal implements OnBoardButtonsIO {
  private final DigitalInput homeButton;
  private final DigitalInput brakeButton;

  public OnBoardButtonsIOReal(int homeButtonPort, int brakeButtonPort) {
    homeButton = new DigitalInput(homeButtonPort);
    brakeButton = new DigitalInput(brakeButtonPort);
  }

  @Override
  public void updateInputs(OnBoardButtonsIOInputs inputs) {
    inputs.homeButtonPressed = !homeButton.get();
    inputs.brakeButtonPressed = !brakeButton.get();
  }
}
