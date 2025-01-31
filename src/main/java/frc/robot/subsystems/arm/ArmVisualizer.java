// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer {
  public static final Translation2d armOrigin2d =
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(6));
  public static final Translation3d armOrigin3d =
      new Translation3d(armOrigin2d.getX(), Units.inchesToMeters(8), armOrigin2d.getY());
  public static final Distance EXTENSION_MIN = Distance.ofBaseUnits(0.6096, Meters);

  private final String name;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d shoulderMechanism;
  private final LoggedMechanismLigament2d wristMechanism;

  public ArmVisualizer(String name) {
    this.name = name;
    LoggedMechanismRoot2d root =
        mechanism.getRoot(name + " Root", armOrigin2d.getX(), armOrigin2d.getY());
    shoulderMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Shoulder",
                Units.inchesToMeters(24.0),
                0.0,
                8.0,
                new Color8Bit(Color.kFirstRed)));
    wristMechanism =
        shoulderMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Wrist",
                Units.inchesToMeters(8.0),
                90.0,
                8.0,
                new Color8Bit(Color.kFirstBlue)));
  }

  public void update(Angle shoulderAngle, Distance extensionLength, Angle wristAngle) {
    shoulderMechanism.setAngle(shoulderAngle.in(Degrees));
    shoulderMechanism.setLength(extensionLength.in(Meters) + EXTENSION_MIN.in(Meters));
    wristMechanism.setAngle(wristAngle.in(Degrees));
    Logger.recordOutput("Arm/Mechanism2d/" + name, mechanism);
    Pose3d armPose = getArmPose(shoulderAngle);
    Pose3d wristPose = getWristPose(shoulderAngle, wristAngle, extensionLength);
    Logger.recordOutput("Arm/Mechanism3d/" + name, armPose, wristPose);
  }

  private Pose3d getArmPose(Angle shoulderAngle) {
    return new Pose3d(
        armOrigin3d.getX(),
        armOrigin3d.getY(),
        armOrigin3d.getZ(),
        new Rotation3d(0, shoulderAngle.abs(Degrees), 0));
  }

  /*
   * Get the pose of the wrist at the end of the arm
   */
  public Pose3d getWristPose(Angle armAngle, Angle wristAngle, Distance armExtension) {
    return getArmPose(armAngle)
        .transformBy(
            new Transform3d(
                new Translation3d(armExtension.abs(Meters), 0, 0),
                new Rotation3d(0, wristAngle.abs(Degrees), 0)));
  }
}
