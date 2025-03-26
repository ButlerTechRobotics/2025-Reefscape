// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ReefDrive.Side;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.Goal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class AutoScore extends SequentialCommandGroup {

  /**
   * Creates a command that automatically aligns to a scoring position, scores, and returns to
   * standby.
   *
   * @param drive The drive subsystem
   * @param arm The arm subsystem
   * @param intake The intake subsystem
   * @param side Which side to align to (LEFT, RIGHT, CENTER)
   * @param goal The arm goal position for scoring
   */
  public AutoScore(Drive drive, Arm arm, Intake intake, Side side, SmartArm.Goal goal) {
    // Step 0: First ensure arm is in standby position
    addCommands(arm.setGoalCommand(Goal.STANDBY));

    // Step 1: Drive to the reef and move arm to position simultaneously
    addCommands(Commands.parallel(new ReefDrive(drive, side), new SmartArm(arm, goal)));

    // Step 2: Score the game piece (coral) - use the right method based on front/back
    if (isBackScoring(goal)) {
      addCommands(intake.scoreCoralFromBack());
    } else {
      addCommands(intake.scoreCoralFromFront());
    }

    // Step 3: Return arm to standby position
    addCommands(new SmartArm(arm, SmartArm.Goal.STANDBY));
  }

  /** Determines if this goal is scored from the back side. */
  private boolean isBackScoring(SmartArm.Goal goal) {
    return goal == SmartArm.Goal.CORAL_L1BACK
        || goal == SmartArm.Goal.CORAL_L2BACK
        || goal == SmartArm.Goal.CORAL_L3BACK
        || goal == SmartArm.Goal.CORAL_L4BACK;
  }
}
