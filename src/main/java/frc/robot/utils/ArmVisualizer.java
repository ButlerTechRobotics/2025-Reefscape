package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {
      private final LoggedMechanism2d mechanism;
      private final LoggedMechanismLigament2d arm;
      private final String key;
      private Distance armLength;

 public ArmVisualizer(String key, Color color, Distance initialArmLength) {
    this.key = key;
    this.armLength = initialArmLength;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm = new LoggedMechanismLigament2d("arm", armLength.abs(Inches), 0.0, 6, new Color8Bit(color));
    root.append(arm);
  }
    
  /** Update arm visualizer with current arm angle */
  public void update(Angle angleRads, Distance armExtension) {
    // Log Mechanism2d
    arm.setAngle(angleRads.abs(Degrees));
    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(Inches.of(0.0), Inches.of(0.0), Inches.of(0.0), new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));
    Logger.recordOutput("Arm/Mechanism3d/" + key, pivot);
  }
}
