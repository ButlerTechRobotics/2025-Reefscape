package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    // Joint motor status
    public boolean jointLeaderConnected = false;
    public boolean jointFollowerConnected = false;
    public boolean jointEncoderConnected = false;

    // Extension motor status
    public boolean extensionLeaderConnected = false;
    public boolean extensionFollowerConnected = false;
    public boolean extensionEncoderConnected = false;

    // Joint positions
    public Angle jointLeaderPosition = Rotations.of(0);
    public Angle jointLeaderRotorPosition = Rotations.of(0);
    public Angle jointEncoderPosition = Rotations.of(0);

    // Extension positions
    public Angle extensionLeaderPosition = Rotations.of(0);
    public Angle extensionLeaderRotorPosition = Rotations.of(0);
    public Angle extensionEncoderPosition = Rotations.of(0);

    // Joint velocities
    public AngularVelocity jointLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity jointLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity jointEncoderVelocity = RotationsPerSecond.of(0);

    // Extension velocities
    public AngularVelocity extensionLeaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity extensionLeaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity extensionEncoderVelocity = RotationsPerSecond.of(0);

    // Joint electrical measurements
    public Voltage jointAppliedVoltage = Volts.of(0.0);
    public Current jointLeaderStatorCurrent = Amps.of(0);
    public Current jointFollowerStatorCurrent = Amps.of(0);
    public Current jointLeaderSupplyCurrent = Amps.of(0);
    public Current jointFollowerSupplyCurrent = Amps.of(0);

    // Extension electrical measurements
    public Voltage extensionAppliedVoltage = Volts.of(0.0);
    public Current extensionLeaderStatorCurrent = Amps.of(0);
    public Current extensionFollowerStatorCurrent = Amps.of(0);
    public Current extensionLeaderSupplyCurrent = Amps.of(0);
    public Current extensionFollowerSupplyCurrent = Amps.of(0);

    // Processed measurements for subsystem use
    public Angle jointAngle = Rotations.of(0);
    public Distance extensionDistance = Inches.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Run closed loop to the specified joint angle and extension distance.
   *
   * @param jointAngle The target joint angle
   * @param extensionDistance The target extension distance
   */
  public default void setPosition(Angle jointAngle, Distance extensionDistance) {}

  /** Stop all motors in open loop. */
  public default void stop() {}
}
