// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AllianceFlipUtil;

public class DriveToReef extends Command {
  Drive drivetrain;
  Side side;
  Command pathCommand;
  Pose2d[] flippedCenterFaces;

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

  // Mapping from pose indices to letters
  private static final String[] poseLetters = {"F", "FL", "BL", "B", "BR", "FR"};

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  public DriveToReef(Drive drivetrain, Side side) {
    this.drivetrain = drivetrain;
    this.side = side;

    // Apply alliance flip to all poses
    this.flippedCenterFaces = new Pose2d[centerFaces.length];
    for (int i = 0; i < centerFaces.length; i++) {
      flippedCenterFaces[i] = AllianceFlipUtil.apply(centerFaces[i]);
    }

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d closestPose = drivetrain.findClosestPose(flippedCenterFaces);
    int closestPoseIndex = getClosestPoseIndex(closestPose, flippedCenterFaces);
    System.out.println("Closest Pose: " + closestPose + ", Index: " + closestPoseIndex);

    pathCommand = getPathCommand(closestPoseIndex, side);
    if (pathCommand != null) {
      pathCommand.schedule();
    } else {
      System.out.println("Path command is null");
    }
  }

  private Command getPathCommand(int closestPoseIndex, Side side) {
    try {
      String pathName = getPathName(closestPoseIndex, side);
      System.out.println("Trying to run path: " + pathName);
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return drivetrain.goToPath(path);
    } catch (Exception e) {
      System.out.println("Path not found: " + e.getMessage());
      return null;
    }
  }

  private String getPathName(int closestPoseIndex, Side side) {
    // Implement logic to determine the path name based on the closest pose index and side
    // For example:
    String poseLetter = poseLetters[closestPoseIndex];
    if (side == Side.LEFT) {
      return "SD-" + poseLetter + "-L";
    } else if (side == Side.CENTER) {
      return "SD-" + poseLetter + "-C";
    } else {
      return "SD-" + poseLetter + "-R";
    }
  }

  private int getClosestPoseIndex(Pose2d closestPose, Pose2d[] poses) {
    for (int i = 0; i < poses.length; i++) {
      if (poses[i].equals(closestPose)) {
        return i;
      }
    }
    return -1; // Should not happen if closestPose is guaranteed to be in poses
  }

  @Override
  public boolean isFinished() {
    return true; // Adjust this based on your command's requirements
  }
}
