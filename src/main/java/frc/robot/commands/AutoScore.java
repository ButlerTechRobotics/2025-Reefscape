package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldConstants.Reef;

public class AutoScore extends Command {
  Drive drivetrain;
  Claw claw;
  Arm arm;
  Side side;
  ArmMode armMode;
  Command pathCommand;

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

  public AutoScore(Drive drivetrain, Claw claw, Arm arm, Side side, ArmMode armMode) {
    this.drivetrain = drivetrain;
    this.claw = claw;
    this.arm = arm;
    this.side = side;
    this.armMode = armMode;

    addRequirements(claw, arm);
  }

  @Override
  public void initialize() {
    Pose2d closestPose = drivetrain.findClosestPose(Reef.centerFaces);
    int closestPoseIndex =
        getClosestPoseIndex(closestPose, Reef.centerFaces) + 1; // Convert to 1-based index
    System.out.println("Closest Pose: " + closestPose + ", Index: " + closestPoseIndex);

    pathCommand = getPathCommand(closestPoseIndex, side);
    if (pathCommand != null) {
      pathCommand.schedule();
    } else {
      System.out.println("Path command is null");
    }

    switch (armMode) {
      case STOP:
        arm.stopCommand().schedule();
        break;
      case INTAKE:
        arm.intake().schedule();
        break;
      case L1:
        arm.L1().schedule();
        break;
      case L2:
        arm.L2().schedule();
        break;
      case L3:
        arm.L3().schedule();
        break;
      case L4:
        arm.L4().schedule();
        break;
    }
  }

  private Command getPathCommand(int closestPoseIndex, Side side) {
    try {
      String pathName = getPathName(closestPoseIndex, side);
      System.out.println("Trying to run path: " + pathName);
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      PathConstraints constraints =
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
      System.out.println("Path constraints: " + constraints);
      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
      System.out.println("Path not found: " + e.getMessage());
      return null;
    }
  }

  private String getPathName(int closestPoseIndex, Side side) {
    // Implement logic to determine the path name based on the closest pose index and side
    // For example:
    if (side == Side.LEFT) {
      return "Left_" + closestPoseIndex;
    } else if (side == Side.CENTER) {
      return "Center_" + closestPoseIndex;
    } else {
      return "Right_" + closestPoseIndex;
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
