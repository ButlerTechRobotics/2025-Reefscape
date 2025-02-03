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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AllianceFlipUtil;

public class SmartScore extends Command {
  Drive drivetrain;
  Claw claw;
  Arm arm;
  Side side;
  ArmMode armMode;
  Command driveCommand;

  // Center faces of the scoring positions
  private static final Pose2d[] centerFaces = {
    AllianceFlipUtil.apply(new Pose2d(3.151, 4.030, Rotation2d.fromDegrees(180))),
    AllianceFlipUtil.apply(new Pose2d(3.151, 5.177, Rotation2d.fromDegrees(120))),
    AllianceFlipUtil.apply(new Pose2d(5.135, 5.177, Rotation2d.fromDegrees(60))),
    AllianceFlipUtil.apply(new Pose2d(5.823, 4.030, Rotation2d.fromDegrees(0))),
    AllianceFlipUtil.apply(new Pose2d(5.135, 2.873, Rotation2d.fromDegrees(-60))),
    AllianceFlipUtil.apply(new Pose2d(3.829, 2.873, Rotation2d.fromDegrees(-120)))
  };

  // Offset distance for left/right positions
  private static final double SIDE_OFFSET = Units.inchesToMeters(6.469);

  public enum ArmMode {
    STOW,
    FLOOR_INTAKE,
    SOURCE_INTAKE,
    L1,
    L2,
    L3,
    L4,
    CLIMB
  }

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  public SmartScore(Drive drivetrain, Arm arm, Side side, ArmMode armMode) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.side = side;
    this.armMode = armMode;

    addRequirements(drivetrain, arm);
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

    // Schedule arm command based on robot orientation
    scheduleArmCommand(useReversedHeading);
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

  private void scheduleArmCommand(boolean isReversed) {
    switch (armMode) {
      case STOW:
        arm.setGoalCommand(Arm.Goal.STOW).schedule();
        break;
      case FLOOR_INTAKE:
        arm.setGoalCommand(Arm.Goal.FLOOR_INTAKE).schedule();
        break;
      case SOURCE_INTAKE:
        arm.setGoalCommand(Arm.Goal.SOURCE_INTAKE).schedule();
        break;
      case L1:
        if (isReversed) {
          arm.setGoalCommand(Arm.Goal.L1).schedule();
        } else {
          arm.setGoalCommand(Arm.Goal.L1Back).schedule();
        }
        break;
      case L2:
        if (isReversed) {
          arm.setGoalCommand(Arm.Goal.L2).schedule();
        } else {
          arm.setGoalCommand(Arm.Goal.L2Back).schedule();
        }
        break;
      case L3:
        if (isReversed) {
          arm.setGoalCommand(Arm.Goal.L3).schedule();
        } else {
          arm.setGoalCommand(Arm.Goal.L3Back).schedule();
        }
        break;
      case L4:
        if (isReversed) {
          arm.setGoalCommand(Arm.Goal.L4).schedule();
        } else {
          arm.setGoalCommand(Arm.Goal.L4Back).schedule();
        }
        break;
      case CLIMB:
        arm.setGoalCommand(Arm.Goal.CLIMB).schedule();
        break;
    }
  }

  @Override
  public void execute() {
    // Continuously check and update the drive command if necessary
    if (driveCommand != null && !driveCommand.isScheduled()) {
      driveCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
