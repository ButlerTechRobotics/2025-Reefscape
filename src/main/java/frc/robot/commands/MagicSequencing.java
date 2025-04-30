// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ReefDrive.Side;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.Goal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Executes a complete scoring sequence combining drive alignment, arm movement, and shooting.
 *
 * <p>The sequence follows these steps: 1. BEGIN PARALLEL ACTIONS: a. Start reef drive alignment
 * (continuously aligns robot with reef on specified side) b. BEGIN SEQUENTIAL ACTIONS (racing with
 * the drive alignment): i. Wait until robot is within 0.006 meters of target reef position ii.
 * Simultaneously shuffle coral to back position to prepare for scoring iii. Simultaneously move arm
 * to STANDBY position and wait until the arm is there iv. Move arm to specified scoring position
 * and wait until the arm is there This parallel group ends when step 1.b completes
 *
 * <p>2. AFTER ALIGNMENT AND POSITIONING: a. Execute AUTO_SHOOT to score the game piece b. Move arm
 * safely to PRE_INTAKE position for the next game piece
 *
 * <p>3. FINALLY: a. Stop the drive motors regardless of sequence outcome
 *
 * @param drive The drive subsystem for reef alignment
 * @param arm The arm subsystem for positioning
 * @param intake The intake subsystem for game piece manipulation
 * @param side Which side of the reef to align with (LEFT or RIGHT)
 * @param scorePos Supplier for the arm goal position (L1, L2, L3, L4, etc.)
 * @param autoShoot Whether to use auto-shooting behavior
 * @return A command that executes the full scoring sequence
 */
public class MagicSequencing {
  public static final Command magicScoreScore(
      Drive drivetrain,
      Arm arm,
      Intake intake,
      Side side,
      Arm.Goal scorePos,
      BooleanSupplier autoShoot) {
    return new ReefDrive(drivetrain, side)
        .raceWith(
            Commands.defer(() -> drivetrain.waitUntilWithinReefDistance(0.006), Set.of())
                .alongWith(intake.shuffleCoralToBack())
                .alongWith(
                    arm.setGoalCommand(Goal.STANDBY)
                        .andThen(Commands.waitUntil(() -> arm.isAtTarget()))))
        .andThen(arm.setGoalAutoCommand(scorePos))
        .andThen(Commands.waitUntil(() -> arm.isAtTarget()))
        .andThen(intake.AUTO_SHOOT())
        .andThen(arm.safeGoToPreIntake())
        .finallyDo(() -> drivetrain.stop());
  }
}
