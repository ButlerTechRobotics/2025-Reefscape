package frc.robot.subsystems.arm_extension;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ArmExtensionIOSIM extends ArmExtensionIOCTRE {

  private final ElevatorSim motorSimModel;
  private final TalonFXSimState leaderSim;

  public ArmExtensionIOSIM() {
    super();
    leaderSim = leader.getSimState();
    DCMotor motor = DCMotor.getKrakenX60Foc(2);
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createElevatorSystem(motor, MASS, DRUM_CIRCUMFERENCE_METERS, GEAR_RATIO);
    motorSimModel =
        new ElevatorSim(linearSystem, motor, MINIMUM_HEIGHT, MAXIMUM_HEIGHT, true, MINIMUM_HEIGHT);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    super.updateInputs(inputs);
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = leaderSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    leaderSim.setRawRotorPosition(
        GEAR_RATIO * Units.radiansToRotations(motorSimModel.getPositionMeters()));
  }
}
