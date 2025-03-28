// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  CANdle candle;
  boolean isIntaking;
  boolean hasGamePiece;
  boolean isAutoAligning;
  int stripLength = 60;
  int startOffset = 0;
  /** Creates new LEDs. */
  public LEDs() {
    candle = new CANdle(14, "CANivore");
    candle.configLEDType(LEDStripType.RGB);
    candle.configV5Enabled(true);
  }

  public void setIsIntaking(boolean isIntaking) {
    this.isIntaking = isIntaking;
  }

  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
  }

  public void setIsAutoAligning(boolean isAutoAligning) {
    this.isAutoAligning = isAutoAligning;
  }

  @Override
  public void periodic() {
    // Prioritize auto-align state
    if (isAutoAligning) {
      // When auto-aligning, always show fire animation regardless of game piece status
      candle.animate(new FireAnimation(69, 0.75, stripLength, 1.0, 0.3));
      return;
    }
    // If not auto-aligning, proceed with game piece state
    if (hasGamePiece) {
      candle.animate(new SingleFadeAnimation(254, 55, 0, 0, 2, stripLength, startOffset));
    } else {
      // Default state when not auto-aligning and no game piece
      candle.animate(new SingleFadeAnimation(254, 55, 0, 0, 0, stripLength, startOffset));
    }
  }
}
