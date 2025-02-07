package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SmartDrive extends Command {
  Drive drivetrain;
  Side side;
  Command pathCommand;

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

  public enum ArmMode {
    STOP,
    INTAKE,
    L1,
    L2,
    L3,
    L4
  }

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
    Pose2d closestPose = drivetrain.findClosestPose(centerFaces);
    int closestPoseIndex =
        getClosestPoseIndex(closestPose, centerFaces) + 1; // Convert to 1-based index
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
    if (side == Side.LEFT) {
      return "Left-" + closestPoseIndex;
    } else if (side == Side.CENTER) {
      return "Center-" + closestPoseIndex;
    } else {
      return "Right-" + closestPoseIndex;
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