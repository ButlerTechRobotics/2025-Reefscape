// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSIM extends ArmIOCTRE {

  private final SingleJointedArmSim armShoulderSimModel;
  private final SingleJointedArmSim armWristSimModel;
  private final ElevatorSim armExtensionSimModel;
  private final TalonFXSimState armShoulderLeaderSim;
  private final TalonFXSimState armWristLeaderSim;
  private final TalonFXSimState armExtensionLeaderSim;

  public ArmIOSIM() {
    super(); // Initialize hardware interface components

    // Get simulation states for all hardware
    armShoulderLeaderSim = shoulderLeader.getSimState();
    armWristLeaderSim = wristLeader.getSimState();
    armExtensionLeaderSim = extensionLeader.getSimState();

    // Configure dual Kraken X60 FOC motors
    DCMotor armShoulderMotor = DCMotor.getKrakenX60Foc(2);

    // Create arm physics model
    LinearSystem<N2, N1, N2> armShoulderLinearSystem =
        LinearSystemId.createSingleJointedArmSystem(armShoulderMotor, 0.00032, SHOULDER_GEAR_RATIO);
    armShoulderSimModel =
        new SingleJointedArmSim(
            armShoulderLinearSystem,
            armShoulderMotor,
            SHOULDER_GEAR_RATIO,
            1,
            -0.785398,
            1.5708,
            true,
            0);

    // Configure single Kraken X60 FOC motor
    DCMotor armWristMotor = DCMotor.getKrakenX60Foc(1);

    // Create arm physics model
    LinearSystem<N2, N1, N2> armWristLinearSystem =
        LinearSystemId.createSingleJointedArmSystem(armWristMotor, 0.00032, WRIST_GEAR_RATIO);
    armWristSimModel =
        new SingleJointedArmSim(
            armWristLinearSystem, armWristMotor, WRIST_GEAR_RATIO, 1, -0.785398, 1.5708, true, 0);

    // Configure dual Kraken X60 FOC motors
    DCMotor armExtensionMotor = DCMotor.getKrakenX60Foc(2);

    // Create elevator physics model
    LinearSystem<N2, N1, N2> armExtensionLinearSystem =
        LinearSystemId.createElevatorSystem(
            armExtensionMotor,
            Pounds.of(10).in(Kilograms), // Carriage mass (10 lbs -> kg)
            extensionRadius.in(Meters), // Drum radius in meters
            EXTENSION_GEAR_RATIO); // Motor to mechanism gear ratio
    armExtensionSimModel =
        new ElevatorSim(
            armExtensionLinearSystem,
            armExtensionMotor,
            0, // Initial position
            Feet.of(8).in(Meters), // Maximum height (8 feet -> meters)
            true, // Enable gravity simulation
            0); // Start at bottom position
  }

  public void updateInputs(ArmIOInputs inputs) {
    // Arm Shoulder Updates
    armShoulderLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var armShoulderMotorVoltage = armShoulderLeaderSim.getMotorVoltage();
    armShoulderSimModel.setInputVoltage(armShoulderMotorVoltage);
    armShoulderSimModel.update(0.020); // Assume 20 ms loop time
    armShoulderLeaderSim.setRawRotorPosition(
        SHOULDER_GEAR_RATIO * Units.radiansToRotations(armShoulderSimModel.getAngleRads()));

    // Arm Wrist Updates
    armWristLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var armWristMotorVoltage = armWristLeaderSim.getMotorVoltage();
    armWristSimModel.setInputVoltage(armWristMotorVoltage);
    armWristSimModel.update(0.020); // Assume 20 ms loop time
    armWristLeaderSim.setRawRotorPosition(
        WRIST_GEAR_RATIO * Units.radiansToRotations(armWristSimModel.getAngleRads()));

    // Arm Extension Updates
    armExtensionLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var armExtensionMotorVoltage = armExtensionLeaderSim.getMotorVoltage();
    armExtensionSimModel.setInputVoltage(armExtensionMotorVoltage);
    armExtensionSimModel.update(0.020); // Assume 20 ms loop time
    armExtensionLeaderSim.setRawRotorPosition(
        EXTENSION_GEAR_RATIO * Units.radiansToRotations(armExtensionSimModel.getPositionMeters()));
  }
}
