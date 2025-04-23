package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
    default void setSetpointInDegrees(double setpointInDegrees) {}

    default void setHomingPosition(double position) {}

    default void disableSoftLimits() {}

    default void updateInputs(ShoulderIOInputs inputs) {}

    default void setPercentage(double percentage) {}

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