// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.config;

import frc.robot.utils.drivers.CanDeviceId;

public class PortConfiguration {
  public int candleID;
  public String CANBus;
  public CanDeviceId flShoulderMotorID;
  public CanDeviceId frShoulderMotorID;
  public CanDeviceId blShoulderMotorID;
  public CanDeviceId brShoulderMotorID;
  public CanDeviceId shoulderEncoderID;
  public CanDeviceId frontExtensionMotorID;
  public CanDeviceId rearExtensionMotorID;
  public CanDeviceId wristMotorID;
  public CanDeviceId wristEncoderID;
  public CanDeviceId intakeMotorID;
  public CanDeviceId frontCANRangeMotorID;
  public CanDeviceId rearCANRangeMotorID;
  public int beamBreakDIOId;

  public PortConfiguration withCandleID(int candleID) {
    this.candleID = candleID;
    return this;
  }

  public PortConfiguration withCANBus(String CANBus) {
    this.CANBus = CANBus;
    return this;
  }
}
