// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOCTRE implements IntakeIO {
  public static final Distance RANGE_FINDER_THRESHOLD = Inches.of(2.4);

  public final TalonFX leader = new TalonFX(42, "CANivore");

  public final CANrange frontCANrange = new CANrange(43, "CANivore");
  public final CANrange backCANrange = new CANrange(44, "CANivore");

  public static final double GEAR_RATIO = 1.0;

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();

  private final StatusSignal<Distance> frontCANrangeDistance = frontCANrange.getDistance();
  private final StatusSignal<Distance> backCANrangeDistance = backCANrange.getDistance();

  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer frontCANrangeConnectedDebounce = new Debouncer(0.5);
  private final Debouncer frontCANrangeDebounce = new Debouncer(0.1, DebounceType.kBoth);
  private final Debouncer backCANrangeConnectedDebounce = new Debouncer(0.5);
  private final Debouncer backCANrangeDebounce = new Debouncer(0.1, DebounceType.kBoth);

  public IntakeIOCTRE() {
    TalonFXConfiguration config = createMotorConfiguration();
    leader.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent);

    leader.optimizeBusUtilization(4, 0.1);
    frontCANrange.optimizeBusUtilization(4, 0.1);
    backCANrange.optimizeBusUtilization(4, 0.1);
  }

  private TalonFXConfiguration createMotorConfiguration() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return config;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    StatusCode frontCANrangeStatus = BaseStatusSignal.refreshAll(frontCANrangeDistance);
    StatusCode backCANrangeStatus = BaseStatusSignal.refreshAll(backCANrangeDistance);

    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.frontCANrangeConnected =
        frontCANrangeConnectedDebounce.calculate(frontCANrangeStatus.isOK());
    inputs.backCANrangeConnected =
        backCANrangeConnectedDebounce.calculate(backCANrangeStatus.isOK());

    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();

    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();

    inputs.frontCANrangeDistance = frontCANrangeDistance.getValue();
    inputs.rearCANrangeDistance = backCANrangeDistance.getValue();

    inputs.hasFrontGamePiece = hasFrontGamePiece();
    inputs.hasBackGamePiece = hasBackGamePiece();
    inputs.hasGamePiece = inputs.hasFrontGamePiece || inputs.hasBackGamePiece;

    // Set the rangeFinderDistance to the sensor that's detecting the game piece
    // This maintains compatibility with the existing code
    if (inputs.hasFrontGamePiece) {
      inputs.rangeFinderDistance = inputs.frontCANrangeDistance;
    } else if (inputs.hasBackGamePiece) {
      inputs.rangeFinderDistance = inputs.rearCANrangeDistance;
    } else {
      inputs.rangeFinderDistance = Inches.of(10); // No game piece detected
    }
  }

  public boolean hasFrontGamePiece() {
    return frontCANrangeDebounce.calculate(
        frontCANrangeDistance.getValue().lt(RANGE_FINDER_THRESHOLD));
  }

  public boolean hasBackGamePiece() {
    return backCANrangeDebounce.calculate(
        backCANrangeDistance.getValue().lt(RANGE_FINDER_THRESHOLD));
  }

  @Override
  public void setVoltage(Voltage volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
