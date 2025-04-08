// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.FieldConstants.ReefHeight;
import org.littletonrobotics.junction.Logger;

public class ReefDrive extends Command {
  private final Drive drivetrain;
  private final Side side;
  private Pose2d targetPose;
  private boolean hasTarget = false;

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  public ReefDrive(Drive drivetrain, Side side) {
    this.drivetrain = drivetrain;
    this.side = side;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Find the closest centerface to current robot pose
    Pose2d currentPose = drivetrain.getPose();
    Pose2d closestCenterface = null;
    int closestIndex = -1;
    double minDistance = Double.MAX_VALUE;

    // Get all centerfaces and compare distances
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      // Apply alliance flip before comparing distances
      Pose2d centerface = AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]);
      double distance = currentPose.getTranslation().getDistance(centerface.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestCenterface = centerface;
        closestIndex = i;
      }
    }

    Logger.recordOutput("ReefDrive/ClosestCenterfaceIndex", closestIndex);

    // Determine target based on side parameter
    if (side == Side.CENTER) {
      // If CENTER, align with the centerface (already alliance-flipped)
      targetPose = closestCenterface;
      hasTarget = true;
    } else {
      // If LEFT or RIGHT, go to the corresponding branch
      int branchIndex;
      if (side == Side.LEFT) {
        // For LEFT side, use the left branch (odd indices in branchPositions)
        branchIndex = closestIndex * 2 + 1;
      } else {
        // For RIGHT side, use the right branch (even indices in branchPositions)
        branchIndex = closestIndex * 2;
      }

      // Get the appropriate branch position at L1 height
      Pose3d reefBranch = FieldConstants.Reef.branchPositions.get(branchIndex).get(ReefHeight.L1);

      // Convert to Pose2d and apply alliance flipping
      Pose2d flippedBranch = AllianceFlipUtil.apply(reefBranch.toPose2d());

      // Apply offset similar to the example in the joystick code
      targetPose =
          flippedBranch.transformBy(
              new Transform2d(
                  Inches.of(2.25).plus(Constants.robotScoringOffset),
                  side == Side.LEFT ? Inches.of(1.8).unaryMinus() : Inches.of(1.8),
                  Rotation2d.k180deg));

      hasTarget = true;
    }

    if (hasTarget) {
      Logger.recordOutput("ReefDrive/TargetPose", targetPose);
    } else {
      Logger.recordOutput("ReefDrive/Error", "Failed to determine target pose");
    }
  }

  @Override
  public void execute() {
    if (hasTarget) {
      // Call the void method directly in execute
      DriveCommands.driveToPointMA(targetPose, drivetrain);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    if (!hasTarget) {
      return true; // End immediately if we don't have a valid target
    }

    // Get current pose
    Pose2d currentPose = drivetrain.getPose();

    // Calculate distance to target
    double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Calculate rotation error, accounting for 180-degree flips
    // Get normal rotation error
    double rotationError =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

    // Get flipped rotation error (if robot is facing backward)
    double flippedRotationError =
        Math.abs(
            currentPose
                .getRotation()
                .plus(new Rotation2d(Math.PI))
                .minus(targetPose.getRotation())
                .getDegrees());

    // Use the smaller of the two errors
    double effectiveRotationError = Math.min(rotationError, flippedRotationError);

    // Log the errors
    Logger.recordOutput("ReefDrive/PositionError", positionError);
    Logger.recordOutput("ReefDrive/NormalRotationError", rotationError);
    Logger.recordOutput("ReefDrive/FlippedRotationError", flippedRotationError);
    Logger.recordOutput("ReefDrive/EffectiveRotationError", effectiveRotationError);

    // Thresholds for determining if we've reached the target
    final double POSITION_THRESHOLD = 0.005; // .5cm position tolerance
    final double ROTATION_THRESHOLD = 0.5; // 0.5 degree rotation tolerance

    // Check if we're within tolerance
    boolean atPosition = positionError < POSITION_THRESHOLD;
    boolean atRotation = effectiveRotationError < ROTATION_THRESHOLD;

    // Log if we've reached target
    Logger.recordOutput("ReefDrive/AtPosition", atPosition);
    Logger.recordOutput("ReefDrive/AtRotation", atRotation);

    if (atPosition && atRotation) {
      System.out.println("ReefDrive: Target reached!");
    }

    // Only finish when both position and rotation are at target
    return atPosition && atRotation;
  }
}
