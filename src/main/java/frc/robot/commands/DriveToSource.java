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

public class DriveToSource extends Command {
  Drive drivetrain;
  Side side;
  Command pathCommand;

  // Source positions
  private static final Pose2d[] sourcePoses = {
    new Pose2d(
        Units.inchesToMeters(0), // CS1
        Units.inchesToMeters(238),
        Rotation2d.fromDegrees(0)),
    new Pose2d(
        Units.inchesToMeters(0), // CS2
        Units.inchesToMeters(275),
        Rotation2d.fromDegrees(0)),
    new Pose2d(
        Units.inchesToMeters(0), // CS3
        Units.inchesToMeters(312),
        Rotation2d.fromDegrees(0))
  };

  // Mapping from pose indices to letters
  private static final String[] poseLetters = {"L", "C", "R"};

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  public DriveToSource(Drive drivetrain, Side side) {
    this.drivetrain = drivetrain;
    this.side = side;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d closestPose = drivetrain.findClosestPose(sourcePoses);
    int closestPoseIndex = getClosestPoseIndex(closestPose, sourcePoses);
    System.out.println("Closest Source: " + closestPose + ", Index: " + closestPoseIndex);

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
    String poseLetter = poseLetters[closestPoseIndex];
    if (side == Side.LEFT) {
      return "SOURCE-" + poseLetter + "-L";
    } else if (side == Side.CENTER) {
      return "SOURCE-" + poseLetter + "-C";
    } else {
      return "SOURCE-" + poseLetter + "-R";
    }
  }

  private int getClosestPoseIndex(Pose2d closestPose, Pose2d[] poses) {
    for (int i = 0; i < poses.length; i++) {
      if (poses[i].equals(closestPose)) {
        return i;
      }
    }
    return -1;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
