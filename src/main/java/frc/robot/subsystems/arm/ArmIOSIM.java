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

  private final SingleJointedArmSim armJointSimModel;
  private final ElevatorSim armExtensionSimModel;
  private final TalonFXSimState armJointLeaderSim;
  private final TalonFXSimState armExtensionLeaderSim;

  public ArmIOSIM() {
    // Arm Joint Simulation
    DCMotor armJointMotor = DCMotor.getKrakenX60Foc(2);
    LinearSystem<N2, N1, N2> armJointLinearSystem =
        LinearSystemId.createSingleJointedArmSystem(armJointMotor, 0.00032, JOINT_GEAR_RATIO);
    armJointSimModel =
        new SingleJointedArmSim(
            armJointLinearSystem, armJointMotor, JOINT_GEAR_RATIO, 1, -0.785398, 1.5708, true, 0);
    armJointLeaderSim = jointLeader.getSimState();

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
    armExtensionLeaderSim = extensionLeader.getSimState();
  }

  public void updateInputs(ArmIOInputs inputs) {
    // Arm Joint Updates
    armJointLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var armJointMotorVoltage = armJointLeaderSim.getMotorVoltage();
    armJointSimModel.setInputVoltage(armJointMotorVoltage);
    armJointSimModel.update(0.020); // Assume 20 ms loop time
    armJointLeaderSim.setRawRotorPosition(
        JOINT_GEAR_RATIO * Units.radiansToRotations(armJointSimModel.getAngleRads()));

    // Arm Extension Updates
    armExtensionLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var armExtensionMotorVoltage = armExtensionLeaderSim.getMotorVoltage();
    armExtensionSimModel.setInputVoltage(armExtensionMotorVoltage);
    armExtensionSimModel.update(0.020); // Assume 20 ms loop time
    armExtensionLeaderSim.setRawRotorPosition(
        EXTENSION_GEAR_RATIO * Units.radiansToRotations(armExtensionSimModel.getPositionMeters()));
  }
}
