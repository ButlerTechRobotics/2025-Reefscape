// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm.shoulder;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of the shoulder subsystem. This class extends ShoulderIOCTRE to provide
 * a physics-based simulation of the shoulder mechanism using WPILib's simulation classes.
 *
 * <p>The simulation models: - Quad Kraken X60 FOC motors - Realistic shoulder physics including
 * gravity and moment of inertia - Position and velocity feedback through simulated encoders -
 * Battery voltage effects - Motion limits (0° to 180°)
 */
public class ShoulderIOSIM extends ShoulderIOCTRE {

  /** Physics simulation model for the shoulder mechanism */
  private final SingleJointedArmSim motorSimModel;

  /** Simulation state for the back-right leader motor */
  private final TalonFXSimState brLeaderSim;
  /** Simulation state for the back-left follower motor */
  private final TalonFXSimState blFollowerSim;
  /** Simulation state for the front-right follower motor */
  private final TalonFXSimState frFollowerSim;
  /** Simulation state for the front-left follower motor */
  private final TalonFXSimState flFollowerSim;
  /** Simulation state for the CANcoder */
  private final CANcoderSimState encoderSim;

  /** Constructs a new ShoulderIOSIM instance. */
  public ShoulderIOSIM() {
    super(); // Initialize hardware interface components

    // Get simulation states for all hardware
    brLeaderSim = brLeader.getSimState();
    blFollowerSim = blFollower.getSimState();
    frFollowerSim = frFollower.getSimState();
    flFollowerSim = flFollower.getSimState();
    encoderSim = encoder.getSimState();

    // Configure quad Kraken X60 FOC motors
    DCMotor motor = DCMotor.getKrakenX60Foc(4);

    // Define shoulder physical properties
    Distance armLength = Inches.of(24);
    Mass armMass = Pounds.of(20);

    // Calculate moment of inertia using WPILib helper
    double armMOI = SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms));

    // Create shoulder physics model
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createSingleJointedArmSystem(motor, armMOI, GEAR_RATIO);

    // Initialize shoulder simulation
    motorSimModel =
        new SingleJointedArmSim(
            linearSystem,
            motor,
            GEAR_RATIO,
            armLength.in(Meters),
            Degrees.of(0).in(Radians), // Lower limit (0°)
            Degrees.of(180).in(Radians), // Upper limit (180)
            true, // Enable gravity simulation
            Degrees.of(0).in(Radians)); // Start at 0°
  }

  /**
   * Overrides the hardware PID configuration to provide simulation-specific tuning.
   *
   * @param config The TalonFXConfiguration to apply PID values to
   * @return The updated configuration with simulation PID values applied
   */
  @Override
  protected TalonFXConfiguration configPID(TalonFXConfiguration config) {
    // Simulation-specific PID values - adjust these based on simulation behavior
    config.Slot0.kP = 620; // Lower P gain for simulation
    config.Slot0.kI = 0; // No integral gain
    config.Slot0.kD = 11; // Lower D gain for simulation
    config.Slot0.kS = 0.08; // Lower static friction compensation
    config.Slot0.kV = 0; // Velocity feedforward
    config.Slot0.kA = 0; // Acceleration feedforward
    config.Slot0.kG = 0.0001; // Lower gravity compensation for simulation
    return config;
  }

  /**
   * Updates the simulation model and all simulated sensor inputs.
   *
   * @param inputs The ShoulderIOInputs object to update with simulated values
   */
  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    // Update base class inputs first
    super.updateInputs(inputs);

    // Simulate battery voltage effects on all devices
    brLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    blFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    frFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    flFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update physics simulation
    motorSimModel.setInputVoltage(brLeaderSim.getMotorVoltage());
    motorSimModel.update(0.020); // Simulate 20ms timestep (50Hz)

    // Get position and velocity from physics simulation
    Angle position = Radians.of(motorSimModel.getAngleRads());
    AngularVelocity velocity = RadiansPerSecond.of(motorSimModel.getVelocityRadPerSec());

    // Update simulated motor encoder readings (accounts for gear ratio)
    brLeaderSim.setRawRotorPosition(position.times(GEAR_RATIO));
    brLeaderSim.setRotorVelocity(velocity.times(GEAR_RATIO));

    // Update simulated CANcoder readings (direct angle measurement)
    encoderSim.setRawPosition(position);
    encoderSim.setVelocity(velocity);
  }
}
