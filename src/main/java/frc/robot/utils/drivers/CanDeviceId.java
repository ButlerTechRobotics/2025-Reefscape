// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.drivers;

public class CanDeviceId {
  private final int deviceNumber;
  private final String bus;

  public CanDeviceId(int deviceNumber, String bus) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
  }

  // Use the default bus name "rio".
  public CanDeviceId(int deviceNumber) {
    this(deviceNumber, "rio");
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBus() {
    return bus;
  }

  @SuppressWarnings("NonOverridingEquals")
  public boolean equals(CanDeviceId other) {
    return other.deviceNumber == deviceNumber && other.bus.equals(bus);
  }

  @Override
  public String toString() {
    return "CanDeviceId(" + deviceNumber + ", " + bus + ")";
  }
}
