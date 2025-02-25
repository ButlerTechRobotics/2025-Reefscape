// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.FieldConstants.ReefHeight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.arm.Arm;

public class AutoScore extends Command {
  private final Drive drivetrain;
  private final Arm arm;
  private final ReefDrive.Side side;
  private final ReefHeight height;
  private Pose2d targetPose;
  private boolean hasTarget = false;
  private boolean isUsingBackSide = false; // Will be determined dynamically

  /**
   * Creates a command to automatically drive to and score at a reef.
   * 
   * @param drivetrain The drive subsystem
   * @param arm The arm subsystem
   * @param side The scoring position (LEFT, CENTER, RIGHT)
   * @param height The scoring height (L1, L2, L3, L4)
   */
  public AutoScore(
      Drive drivetrain, 
      Arm arm, 
      ReefDrive.Side side, 
      ReefHeight height) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.side = side;
    this.height = height;
    
    addRequirements(drivetrain);
    // Note: Not requiring arm since SmartArm will handle that
  }
  
  /**
   * Determines the appropriate SmartArm goal based on height and orientation
   */
  private SmartArm.Goal determineArmGoal() {
    // Map ReefHeight to the appropriate SmartArm.Goal based on whether using back side
    if (!isUsingBackSide) {
      // Forward-facing goals
      switch (height) {
        case L1:
          return SmartArm.Goal.CORAL_L1;
        case L2:
          return SmartArm.Goal.CORAL_L2;
        case L3:
          return SmartArm.Goal.CORAL_L3;
        case L4:
          // No L4 forward, default to L3
          System.out.println("Warning: L4 forward not available, using L3");
          return SmartArm.Goal.CORAL_L3;
        default:
          return SmartArm.Goal.STOW;
      }
    } else {
      // Backward-facing goals
      switch (height) {
        case L1:
          return SmartArm.Goal.CORAL_L1BACK;
        case L2:
          return SmartArm.Goal.CORAL_L2BACK;
        case L3:
          return SmartArm.Goal.CORAL_L3BACK;
        case L4:
          return SmartArm.Goal.CORAL_L4BACK;
        default:
          return SmartArm.Goal.STOW;
      }
    }
  }

  @Override
  public void initialize() {
    // Find the closest centerface to current robot pose
    Pose2d currentPose = drivetrain.getPose();
    Pose2d closestCenterface = null;
    int closestIndex = -1;
    double minDistance = 4.0;
    
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      Pose2d centerface = FieldConstants.Reef.centerFaces[i];
      double distance = currentPose.getTranslation().getDistance(centerface.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestCenterface = centerface;
        closestIndex = i;
      }
    }
    
    System.out.println("Closest Centerface Index: " + closestIndex);
    
    // Determine target based on side parameter
    if (side == ReefDrive.Side.CENTER) {
      // If CENTER, align with the centerface
      targetPose = closestCenterface;
      hasTarget = true;
    } else {
      // If LEFT or RIGHT, go to the corresponding branch
      int branchIndex;
      if (side == ReefDrive.Side.LEFT) {
        // For LEFT side, use the left branch (odd indices in branchPositions)
        branchIndex = closestIndex * 2 + 1;
      } else {
        // For RIGHT side, use the right branch (even indices in branchPositions)
        branchIndex = closestIndex * 2;
      }
      
      // Get the appropriate branch position at the specified height
      Pose3d reefBranch = FieldConstants.Reef.branchPositions.get(branchIndex).get(height);
      
      // Apply offset similar to the example in the joystick code
      targetPose = reefBranch
          .toPose2d()
          .transformBy(
              new Transform2d(
                  Inches.of(2.25).plus(Constants.robotScoringOffset),
                  side == ReefDrive.Side.LEFT ? Inches.of(1.8).unaryMinus() : Inches.of(1.8),
                  Rotation2d.fromDegrees(0))); // Neutral rotation, back/front will be determined dynamically
      
      hasTarget = true;
    }
    
    if (!hasTarget) {
      System.out.println("Failed to determine target pose");
    } else {
      System.out.println("Target Pose: " + targetPose);
      System.out.println("Scoring at height: " + height);
      
      // Determine whether to use back or front based on current robot pose
      isUsingBackSide = DriveCommands.isBackSideCloser(
          drivetrain.getPose().getRotation(), 
          targetPose.getRotation());
      
      System.out.println("Using robot back side: " + isUsingBackSide);
      
      // Deploy arm immediately with the correct goal
      SmartArm.Goal armGoal = determineArmGoal();
      new SmartArm(arm, armGoal).schedule();
      System.out.println("Deploying arm with goal: " + armGoal);
    }
  }

  @Override
  public void execute() {
    if (hasTarget) {
      // Drive to the point using the driveToPointMA method which automatically 
      // determines whether to use front or back of robot
      DriveCommands.driveToPointMA(targetPose, drivetrain, Meters.of(0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends
    drivetrain.stop();
    
    if (interrupted) {
      // If interrupted, make sure the arm is in a safe position
      new SmartArm(arm, SmartArm.Goal.STOW).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    // Since we're directly calling driveToPointMA in execute,
    // we should let the command continue running until it's manually ended
    return false;
  }
  
  /**
   * Factory method to create a command for scoring at L1 height
   */
  public static AutoScore scoreAtL1(Drive drivetrain, Arm arm, ReefDrive.Side side) {
    return new AutoScore(drivetrain, arm, side, ReefHeight.L1);
  }
  
  /**
   * Factory method to create a command for scoring at L2 height
   */
  public static AutoScore scoreAtL2(Drive drivetrain, Arm arm, ReefDrive.Side side) {
    return new AutoScore(drivetrain, arm, side, ReefHeight.L2);
  }
  
  /**
   * Factory method to create a command for scoring at L3 height
   */
  public static AutoScore scoreAtL3(Drive drivetrain, Arm arm, ReefDrive.Side side) {
    return new AutoScore(drivetrain, arm, side, ReefHeight.L3);
  }
  
  /**
   * Factory method to create a command for scoring at L4 height
   * Note: L4 is only available when using the back side of the robot
   */
  public static AutoScore scoreAtL4(Drive drivetrain, Arm arm, ReefDrive.Side side) {
    return new AutoScore(drivetrain, arm, side, ReefHeight.L4);
  }
}