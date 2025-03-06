// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.shoulder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  @AutoLog
  public static class ShoulderIOInputs {
    // Connection status for all motors
    public boolean brLeaderConnected = false;
    public boolean blFollowerConnected = false;
    public boolean frFollowerConnected = false;
    public boolean flFollowerConnected = false;
    public boolean encoderConnected = false;

    /** Current neutral mode. True = brake, False = coast */
    public boolean brakeMode = true;

    /** Current disable state. True = disabled, False = enabled */
    public boolean disableOverride = false;

    // Position measurements
    public Angle brLeaderPosition = Rotations.of(0);
    public Angle brLeaderRotorPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    // Velocity measurements
    public AngularVelocity brLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity brLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    // Voltage and current measurements
    public Voltage appliedVoltage = Volts.of(0.0);

    // Stator currents for all motors
    public Current brLeaderStatorCurrent = Amps.of(0);
    public Current blFollowerStatorCurrent = Amps.of(0);
    public Current frFollowerStatorCurrent = Amps.of(0);
    public Current flFollowerStatorCurrent = Amps.of(0);

    // Supply currents for all motors
    public Current brLeaderSupplyCurrent = Amps.of(0);
    public Current blFollowerSupplyCurrent = Amps.of(0);
    public Current frFollowerSupplyCurrent = Amps.of(0);
    public Current flFollowerSupplyCurrent = Amps.of(0);

    // Derived angle measurement
    public Angle shoulderAngle = Rotations.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShoulderIOInputs inputs) {}

  /** Run closed loop at the specified position. */
  public default void setPosition(Angle angle) {}

  /** Run open loop at the specified voltage. */
  default void runVolts(double volts) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}

  public default void setMaxVelocity(AngularVelocity maxVelocity) {}

  public default void setMaxAcceleration(double maxAcceleration) {}

  /**
   * Sets the encoder position to the specified angle.
   *
   * @param position The position to set the encoder to
   */
  public default void setEncoderPosition(Angle position) {}

  /**
   * Gets the current angular velocity of the shoulder mechanism.
   *
   * @return The current angular velocity
   */
  public default AngularVelocity getVelocity() {
    return RotationsPerSecond.of(0);
  }

  /** Stop in open loop. */
  public default void stop() {}

  /**
   * Sets the neutral mode of the motors.
   *
   * @param brake True for brake mode, false for coast mode
   */
  public default void setBrakeMode(boolean brake) {}

  /**
   * Sets the override for the disable state.
   *
   * @param disabled True to disable the motors, false to enable them
   */
  public default void setDisableOverride(boolean disabled) {}
  ;
}
