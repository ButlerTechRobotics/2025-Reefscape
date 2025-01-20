package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldConstants.Reef;

public class AutoScore extends Command {
  Drive drive;
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

  public AutoScore(Drive drive, Claw claw, Arm arm, Side side, ArmMode armMode) {
    this.drive = drive;
    this.claw = claw;
    this.arm = arm;
    this.side = side;
    this.armMode = armMode;

    addRequirements(claw, arm);
  }

  @Override
  public void initialize() {
    Pose2d closestPose = drive.findClosestPose(Reef.centerFaces);
    System.out.println("Closest Pose: " + closestPose);

    pathCommand = getPathCommand(closestPose, side);
    pathCommand.schedule();

    switch (armMode) {
      case STOP:
        arm.stopCommand();
        break;
      case INTAKE:
        arm.intake();
        break;
      case L1:
        arm.L1();
        break;
      case L2:
        arm.L2();
        break;
      case L3:
        arm.L3();
        break;
    }
  }

  private Command getPathCommand(Pose2d closestPose, Side side) {
    try {
      String pathName = getPathName(closestPose, side);
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      PathConstraints constraints =
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
      System.out.println("Path not found");
      return Commands.none();
    }
  }

  private String getPathName(Pose2d closestPose, Side side) {
    // Implement logic to determine the path name based on the closest pose and side
    // For example:
    if (side == Side.LEFT) {
      return "Left";
    } else if (side == Side.CENTER) {
      return "Center";
    } else {
      return "Right";
    }
  }

  @Override
  public boolean isFinished() {
    return true; // Adjust this based on your command's requirements
  }
}
