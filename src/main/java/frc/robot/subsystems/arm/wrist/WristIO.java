// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public boolean encoderConnected = false;

    /** Current neutral mode. True = brake, False = coast */
    public boolean brakeMode = true;

    /** Current disable state. True = disabled, False = enabled */
    public boolean disableOverride = false;

    public Angle leaderPosition = Rotations.of(0);
    public Angle leaderRotorPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity leaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);

    public Angle wristAngle = Rotations.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(Angle angle) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}

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
