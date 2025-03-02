// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ClawMode;

public class SmartIntake extends Command {

  private final Intake intake;
  private final BeamBreak beamBreak;
  private final ClawMode goal;
  private final double outtakeDelay;

  /**
   * Creates a new SmartIntake.
   *
   * @param intake The intake subsystem.
   * @param beamBreak The beam break subsystem.
   * @param goal The desired intake goal (intake or outtake).
   * @param outtakeDelay The delay for outtake in seconds.
   */
  public SmartIntake(Intake intake, BeamBreak beamBreak, ClawMode goal, double outtakeDelay) {
    this.intake = intake;
    this.beamBreak = beamBreak;
    this.goal = goal;
    this.outtakeDelay = outtakeDelay;
    addRequirements(intake);
  }

  /**
   * Creates a new SmartIntake with no outtake delay.
   *
   * @param intake The intake subsystem.
   * @param beamBreak The beam break subsystem.
   * @param goal The desired intake goal (intake or outtake).
   */
  public SmartIntake(Intake intake, BeamBreak beamBreak, ClawMode goal) {
    this(intake, beamBreak, goal, 0.5); // Default delay of 0.5 seconds
  }

  @Override
  public void initialize() {
    intake.setClawMode(goal); // Start the intake/outtake
  }

  @Override
  public void execute() {
    // The intake motor will run until the end condition is met, which is handled in isFinished.
  }

  @Override
  public boolean isFinished() {
    if (goal == ClawMode.FLOOR_INTAKE
        || goal == ClawMode.STATION_INTAKE
        || goal == ClawMode.ALGAE_INTAKE) {
      // Intake goal: finish when game piece is detected
      return beamBreak.hasGamePiece();
    } else if (goal == ClawMode.OUTTAKE) {
      // Outtake goal: finish when game piece is not detected
      return !beamBreak.hasGamePiece();
    } else {
      // Should not happen, but stop if goal is NONE
      return true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (goal == ClawMode.OUTTAKE && !interrupted) {
      // Add a delay after outtake
      new SequentialCommandGroup(new WaitCommand(outtakeDelay), intake.stopCommand()).schedule();
    } else {
      intake.stopCommand().schedule();
    }
  }
}
