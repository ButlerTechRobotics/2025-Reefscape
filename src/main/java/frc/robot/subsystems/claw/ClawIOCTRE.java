// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClawIOCTRE implements ClawIO {
  public static final double GEAR_RATIO = 1.5;

  public final TalonFX leader = new TalonFX(20);

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();

  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer encoderDebounce = new Debouncer(0.5);

  public ClawIOCTRE() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Slot0.kP = 5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    leader.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());

    inputs.leaderPosition = leaderPosition.getValue().div(GEAR_RATIO);
    inputs.leaderVelocity = leaderVelocity.getValue().div(GEAR_RATIO);

    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();

    leader.optimizeBusUtilization(4, 0.1);
  }

  @Override
  public void setVoltage(Voltage volts) {
    leader.setControl(m_voltReq.withOutput(volts.in(Volts)));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
