// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shoulder;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  default void setSetpointInDegrees(double setpointInDegrees) {}

  default void setHomingPosition(double position) {}

  default void disableSoftLimits() {}

  default void enableSoftLimits() {}

  default void updateInputs(ShoulderIOInputs inputs) {}

  default void setPercentage(double percentage) {}

  /**
   * Gets a supplier for a specific signal value. Useful for asynchronous signal processing.
   *
   * @param signalType The type of signal to get
   * @return A supplier function that returns the latest signal value
   */
  default Supplier<Double> getSignalSupplier(SignalType signalType) {
    return () -> 0.0;
  }

  /** Signal types that can be processed asynchronously */
  enum SignalType {
    POSITION,
    VELOCITY,
    VOLTAGE,
    CURRENT,
    TEMPERATURE,
    ENCODER_POSITION,
    ENCODER_VELOCITY
  }

  @AutoLog
  class ShoulderIOInputs {
    public boolean flConnected = true;
    public boolean frConnected = true;
    public boolean blConnected = true;
    public boolean brConnected = true;
    public boolean encoderConnected = true;
    public double shoulderVoltage = 0.0;
    public double shoulderCurrent = 0.0;
    public double shoulderTemperature = 0.0;
    public double shoulderPositionDegrees = 0.0;
    public double shoulderVelocityDegrees = 0.0;
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
  }
}
