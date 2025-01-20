package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d wristLigament;
  private final String key;
  private Distance armLength;

  public ArmVisualizer(String key, Color color, Distance initialArmLength) {
    this.key = key;
    this.armLength = initialArmLength;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("Pivot", 1.0, 0.4);
    armLigament =
        new LoggedMechanismLigament2d("Arm", armLength.abs(Inches), 0.0, 6, new Color8Bit(color));
    root.append(armLigament);
    wristLigament =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "Wrist", Inches.of(10).abs(Meters), 0.0, 6, new Color8Bit(color)));
  }

  /** Update arm visualizer with current arm angle */
  public void update(Angle armAngle, Angle wristAngle, Distance armExtension) {
    // Log Mechanism2d
    armLigament.setAngle(armAngle.abs(Degrees));
    wristLigament.setAngle(wristAngle.abs(Degrees));

    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);

    Pose3d armPose = getArmPose(armAngle.abs(Degrees));
    Pose3d wristPose = getWristPose(armAngle.abs(Degrees), wristAngle.abs(Degrees));

    Logger.recordOutput("Arm/Mechanism3d/" + key, armPose, wristPose);
  }

  Translation2d armRoot = new Translation2d(-0.31, 0.64);

  private Pose3d getArmPose(double armAngle) {
    return new Pose3d(armRoot.getX(), 0, armRoot.getY(), new Rotation3d(0, -armAngle, 0));
  }

  /*
   * Get the pose of the wrist at the end of the arm
   */
  public Pose3d getWristPose(double armAngle, double wristAngle) {
    return getArmPose(armAngle)
        .transformBy(
            new Transform3d(armLength.abs(Inches), 0, 0, new Rotation3d(0, wristAngle, 0)));
  }
}
