// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;

public class DynamicArm extends Command {
  Drive drivetrain;
  Arm arm;
  TargetPose targetPose;

  public enum TargetPose {
    CS1,
    CS2,
    CS3,
    CS4,
    CS5,
    CS6
  }

  // Mapping of coral station positions to 3D poses
  private static final Map<TargetPose, Pose3d> CS_POSES =
      Map.of(
          TargetPose.CS1,
              new Pose3d(
                  0.301,
                  7.030,
                  Units.inchesToMeters(37.5), // Height
                  new Rotation3d(0, 0, 0)),
          TargetPose.CS2,
              new Pose3d(
                  Units.inchesToMeters(160.373),
                  Units.inchesToMeters(186.857),
                  Units.inchesToMeters(24.0),
                  new Rotation3d(0, 0, Math.toRadians(120))),
          TargetPose.CS3,
              new Pose3d(
                  Units.inchesToMeters(193.116),
                  Units.inchesToMeters(186.858),
                  Units.inchesToMeters(24.0),
                  new Rotation3d(0, 0, Math.toRadians(60))),
          TargetPose.CS4,
              new Pose3d(
                  Units.inchesToMeters(209.489 + 18.885),
                  Units.inchesToMeters(158.500),
                  Units.inchesToMeters(24.0),
                  new Rotation3d(0, 0, 0)),
          TargetPose.CS5,
              new Pose3d(
                  Units.inchesToMeters(193.118),
                  Units.inchesToMeters(130.145),
                  Units.inchesToMeters(24.0),
                  new Rotation3d(0, 0, Math.toRadians(-60))),
          TargetPose.CS6,
              new Pose3d(
                  Units.inchesToMeters(160.375),
                  Units.inchesToMeters(130.144),
                  Units.inchesToMeters(24.0),
                  new Rotation3d(0, 0, Math.toRadians(-120))));

  // Constants for arm geometry
  private static final double BASE_HEIGHT = 6.0; // Height of arm base from ground in inches
  private static final double MIN_EXTENSION = 0.0; // Minimum extension length in inches
  private static final double MAX_EXTENSION =
      Units.inchesToMeters(24.0); // Maximum extension length converted to meters
  private static final double WRIST_LENGTH =
      Units.inchesToMeters(8.0); // Length of wrist segment in meters
  private static final double MIN_ANGLE = 0.0; // Minimum shoulder angle in degrees
  private static final double MAX_ANGLE = 90.0; // Maximum shoulder angle in degrees

  public DynamicArm(Drive drivetrain, Arm arm, TargetPose targetPose) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.targetPose = targetPose;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    updateArmPosition();
  }

  @Override
  public void execute() {
    updateArmPosition();
  }

  private void updateArmPosition() {
    Pose2d currentPose = drivetrain.getPose();
    Pose3d targetPose3d = CS_POSES.get(targetPose);

    // Calculate distances
    double dx = targetPose3d.getX() - currentPose.getX();
    double dy = targetPose3d.getY() - currentPose.getY();
    double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

    // Subtract base height from target height
    double verticalDistance = targetPose3d.getZ() - Units.inchesToMeters(BASE_HEIGHT);

    // Calculate total required reach (extension + wrist)
    double requiredLength =
        Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);

    // Subtract wrist length to get extension length
    double extensionLength = requiredLength - WRIST_LENGTH;

    double requiredAngle = Math.toDegrees(Math.atan2(verticalDistance, horizontalDistance));

    // Clamp values
    double clampedLength = clamp(extensionLength, MIN_EXTENSION, MAX_EXTENSION);
    double clampedAngle = clamp(requiredAngle, MIN_ANGLE, MAX_ANGLE);

    // Set arm position
    arm.getExtension().setLength(Meters.of(clampedLength));
    arm.getShoulder().setAngle(Degrees.of(clampedAngle));
    arm.getWrist().setAngle(Degrees.of(0));
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    arm.setGoalCommand(Arm.Goal.STOW).schedule();
  }
}
