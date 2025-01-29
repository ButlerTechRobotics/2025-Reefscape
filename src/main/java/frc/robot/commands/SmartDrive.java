// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SmartDrive extends Command {
  private Drive drivetrain;
  private Side side;
  private Command driveCommand;

  // Center faces of the scoring positions
  private static final Pose2d[] centerFaces = {
    new Pose2d(
        Units.inchesToMeters(144.003 - 18.885),
        Units.inchesToMeters(158.500),
        Rotation2d.fromDegrees(180)),
    new Pose2d(
        Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
    new Pose2d(
        Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
    new Pose2d(
        Units.inchesToMeters(209.489 + 18.885),
        Units.inchesToMeters(158.500),
        Rotation2d.fromDegrees(0)),
    new Pose2d(
        Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
    new Pose2d(
        Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120))
  };

  // Offset distance for left/right positions
  private static final double SIDE_OFFSET = Units.inchesToMeters(6.469);

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  public SmartDrive(Drive drivetrain, Side side) {
    this.drivetrain = drivetrain;
    this.side = side;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Find closest center face
    Pose2d closestCenterPose = drivetrain.findClosestPose(centerFaces);

    // Get target pose based on side
    Pose2d targetPose = getTargetPose(closestCenterPose, side);

    // Get current robot pose
    Pose2d currentPose = drivetrain.getPose();

    // Calculate angle to target
    double angleToTarget =
        Math.toDegrees(
            Math.atan2(
                targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));

    // Get the relative angle between robot's current heading and angle to target
    double relativeAngle = angleToTarget - currentPose.getRotation().getDegrees();
    // Normalize to -180 to 180
    relativeAngle = (relativeAngle + 180) % 360 - 180;

    // Determine if we should use reversed heading based on relative angle
    boolean useReversedHeading = Math.abs(relativeAngle) < 90;

    // Create final pose with the correct heading
    Rotation2d finalHeading =
        useReversedHeading
            ? Rotation2d.fromDegrees(closestCenterPose.getRotation().getDegrees() + 180)
            : closestCenterPose.getRotation();

    Pose2d finalPose = new Pose2d(targetPose.getX(), targetPose.getY(), finalHeading);

    // Create pathfinding command to the target pose
    driveCommand = drivetrain.goToPoint(finalPose);
    if (driveCommand != null) {
      driveCommand.schedule();
      System.out.println(
          "Using "
              + (useReversedHeading ? "back" : "front")
              + " | Relative angle: "
              + relativeAngle);
    }
  }

  private Pose2d getTargetPose(Pose2d centerPose, Side side) {
    switch (side) {
      case LEFT:
        return centerPose.transformBy(new Transform2d(0, -SIDE_OFFSET, new Rotation2d()));
      case RIGHT:
        return centerPose.transformBy(new Transform2d(0, SIDE_OFFSET, new Rotation2d()));
      case CENTER:
      default:
        return centerPose;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
