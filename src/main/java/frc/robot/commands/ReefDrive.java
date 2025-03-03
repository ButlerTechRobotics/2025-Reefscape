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

    // Get alliance-flipped centerfaces for comparison
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      // Apply alliance flip before comparing distances
      Pose2d centerface = FieldConstants.Reef.centerFaces[i];
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

      // Get the appropriate branch position at L1 height and alliance-flip it
      Pose3d reefBranch = FieldConstants.Reef.branchPositions.get(branchIndex).get(ReefHeight.L1);
      Pose2d flippedBranch = reefBranch.toPose2d();

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
    // Since we're directly calling driveToPointMA in execute,
    // we should let the command continue running until it's manually ended
    return false;
  }
}
