package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private Distance initialArmLength;

  public ArmVisualizer(String key, Color color1, Color color2, Distance initialArmLength) {
    this.key = key;
    this.initialArmLength = initialArmLength;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("Pivot", 1.0, 0.4);
    armLigament =
        new LoggedMechanismLigament2d(
            "Arm", initialArmLength.abs(Meters), 0.0, 8, new Color8Bit(color1));
    root.append(armLigament);
    wristLigament =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "Wrist", Inches.of(4).abs(Meters), 0.0, 4, new Color8Bit(color2)));
  }

  /** Update arm visualizer with current arm angle */
  public void update(Angle armAngle, Angle wristAngle, Distance armExtension) {
    // Log Mechanism2d
    armLigament.setAngle(armAngle.abs(Radians));
    wristLigament.setAngle(wristAngle.abs(Radians));
    armLigament.setLength(initialArmLength.abs(Meters) + armExtension.times(0.0254).abs(Meters));

    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);

    Pose3d armPose = getArmPose(armAngle.abs(Radians));
    Pose3d wristPose = getWristPose(armAngle.abs(Radians), wristAngle.abs(Radians), armExtension);

    Logger.recordOutput("Arm/Mechanism3d/" + key, armPose, wristPose);
  }

  Translation2d armRoot = new Translation2d(-0.31, 0.64);

  private Pose3d getArmPose(double armAngle) {
    return new Pose3d(armRoot.getX(), 0, armRoot.getY(), new Rotation3d(0, -armAngle, 0));
  }

  /*
   * Get the pose of the wrist at the end of the arm
   */
  public Pose3d getWristPose(double armAngle, double wristAngle, Distance armExtension) {
    return getArmPose(armAngle)
        .transformBy(
            new Transform3d(
                new Translation3d(armExtension.abs(Inches), 0, 0),
                new Rotation3d(0, wristAngle, 0)));
  }
}
