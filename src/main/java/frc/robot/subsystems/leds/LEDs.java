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
    // This method will be called once per scheduler run
    // if (DriverStation.isDisabled()) {
    //   // if (aprilTags.getPoseEstimationCount() == 0 || DriverStation.getAlliance().isEmpty()) {
    //   if (DriverStation.getAlliance().isEmpty()) {
    //     candle.animate(new SingleFadeAnimation(0, 100, 0, 0, 0.3, stripLength, startOffset));
    //     return;
    //   }
    //   if (DriverStation.getAlliance().get() == Alliance.Red) {
    //     candle.animate(
    //         new LarsonAnimation(150, 0, 0, 0, 0.05, stripLength, BounceMode.Front, 7, 0));
    //   } else {
    //     candle.animate(
    //         new LarsonAnimation(0, 0, 150, 0, 0.05, stripLength, BounceMode.Front, 7, 0));
    //   }
    //   return;
    // }
    if (hasGamePiece) {
      candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 2, stripLength, startOffset));
    } else {
      if (isAutoAligning) {
        candle.animate(new FireAnimation(100, 0.75, stripLength, 1.0, 0.3));
      } else {
        candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0, stripLength, startOffset));
      }
      return;
    }
  }
}
