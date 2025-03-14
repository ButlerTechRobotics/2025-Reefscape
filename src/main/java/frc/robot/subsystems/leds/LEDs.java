// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  CANdle candle;

  boolean isIntaking;
  boolean hasGamePiece;
  int stripLength = 100;
  int startOffset = 0;

  /** Creates new LEDs. */
  public LEDs() {
    candle = new CANdle(14, "CANivore");
    candle.configLEDType(LEDStripType.GRB);
    candle.configV5Enabled(false);
  }

  public void setIsIntaking(boolean isIntaking) {
    this.isIntaking = isIntaking;
  }

  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
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
      candle.animate(new SingleFadeAnimation(0, 255, 0, 0, 1, stripLength, startOffset));
    } else {
      candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 1, stripLength, startOffset));
    }
  }
}
