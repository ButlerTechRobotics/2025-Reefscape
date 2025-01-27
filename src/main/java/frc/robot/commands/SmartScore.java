package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drive;

public class SmartScore extends Command {
  Drive drivetrain;
  Claw claw;
  Arm arm;
  Side side;
  ArmMode armMode;
  Command driveCommand;

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

  public SmartScore(Drive drivetrain, Claw claw, Arm arm, Side side, ArmMode armMode) {
    this.drivetrain = drivetrain;
    this.claw = claw;
    this.arm = arm;
    this.side = side;
    this.armMode = armMode;

    addRequirements(drivetrain, claw, arm);
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
      case STOP:
        arm.stopCommand().schedule();
        break;
      case INTAKE:
        arm.intake().schedule();
        break;
      case L1:
        if (isReversed) {
          arm.L1Back().schedule();
        } else {
          arm.L1().schedule();
        }
        break;
      case L2:
        if (isReversed) {
          arm.L2Back().schedule();
        } else {
          arm.L2().schedule();
        }
        break;
      case L3:
        if (isReversed) {
          arm.L3Back().schedule();
        } else {
          arm.L3().schedule();
        }
        break;
      case L4:
        if (isReversed) {
          arm.L4Back().schedule();
        } else {
          arm.L4().schedule();
        }
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
