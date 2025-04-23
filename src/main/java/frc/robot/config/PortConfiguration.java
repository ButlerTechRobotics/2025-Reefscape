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