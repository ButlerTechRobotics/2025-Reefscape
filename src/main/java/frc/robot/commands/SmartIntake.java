// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ClawMode;
import org.littletonrobotics.junction.Logger;

public class SmartIntake extends Command {

  private final Intake intake;
  private final BeamBreak beamBreak;
  private final ClawMode goal;
  private final double outtakeDelay;
  private final double intakeDelay; // New delay parameter for intake
  private final Timer timer = new Timer();
  private double detectionTime = -1; // Time when game piece was first detected

  /**
   * Creates a new SmartIntake with both intake and outtake delays.
   *
   * @param intake The intake subsystem.
   * @param beamBreak The beam break subsystem.
   * @param goal The desired intake goal (intake or outtake).
   * @param outtakeDelay The delay for outtake in seconds.
   * @param intakeDelay The delay to continue running intake after game piece detection.
   */
  public SmartIntake(
      Intake intake, BeamBreak beamBreak, ClawMode goal, double outtakeDelay, double intakeDelay) {
    this.intake = intake;
    this.beamBreak = beamBreak;
    this.goal = goal;
    this.outtakeDelay = outtakeDelay;
    this.intakeDelay = intakeDelay;
    addRequirements(intake);
  }

  /**
   * Creates a new SmartIntake with default delays.
   *
   * @param intake The intake subsystem.
   * @param beamBreak The beam break subsystem.
   * @param goal The desired intake goal (intake or outtake).
   */
  public SmartIntake(Intake intake, BeamBreak beamBreak, ClawMode goal) {
    this(intake, beamBreak, goal, 0.0, 0.3); // Default outtake delay of 0.0, intake delay of 0.3
  }

  /**
   * Creates a new SmartIntake with custom outtake delay and default intake delay.
   *
   * @param intake The intake subsystem.
   * @param beamBreak The beam break subsystem.
   * @param goal The desired intake goal (intake or outtake).
   * @param outtakeDelay The delay for outtake in seconds.
   */
  public SmartIntake(Intake intake, BeamBreak beamBreak, ClawMode goal, double outtakeDelay) {
    this(intake, beamBreak, goal, outtakeDelay, 0.3); // Default intake delay of 0.3 seconds
  }

  @Override
  public void initialize() {
    intake.setClawMode(goal); // Start the intake/outtake
    timer.reset();
    timer.start();
    detectionTime = -1; // Reset detection time
    Logger.recordOutput("SmartIntake/Goal", goal.toString());
    Logger.recordOutput("SmartIntake/HasGamePiece", beamBreak.hasGamePiece());
  }

  @Override
  public void execute() {
    // Check if we just detected a game piece
    if (detectionTime < 0
        && beamBreak.hasGamePiece()
        && (goal == ClawMode.FLOOR_INTAKE
            || goal == ClawMode.STATION_INTAKE
            || goal == ClawMode.ALGAE_INTAKE)) {
      detectionTime = timer.get();
      Logger.recordOutput("SmartIntake/GamePieceDetectedTime", detectionTime);
    }

    // Log status to help with debugging
    Logger.recordOutput("SmartIntake/ElapsedTime", timer.get());
    Logger.recordOutput("SmartIntake/HasGamePiece", beamBreak.hasGamePiece());
    Logger.recordOutput(
        "SmartIntake/IntakeDelayRemaining",
        detectionTime >= 0 ? Math.max(0, (detectionTime + intakeDelay) - timer.get()) : -1);
  }

  @Override
  public boolean isFinished() {
    if (goal == ClawMode.FLOOR_INTAKE
        || goal == ClawMode.STATION_INTAKE
        || goal == ClawMode.ALGAE_INTAKE) {
      // Intake goal: finish when game piece is detected AND additional delay has elapsed
      if (beamBreak.hasGamePiece()) {
        // If we've detected a game piece, check if we've run for the additional delay
        return detectionTime >= 0 && (timer.get() >= detectionTime + intakeDelay);
      }
      return false; // No game piece detected yet
    } else if (goal == ClawMode.OUTTAKE) {
      // Outtake goal: finish when game piece is not detected OR timeout elapsed
      return !beamBreak.hasGamePiece() || timer.hasElapsed(outtakeDelay);
    } else {
      // Should not happen, but stop if goal is NONE
      return true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    boolean timedOut = goal == ClawMode.OUTTAKE && timer.hasElapsed(outtakeDelay);
    Logger.recordOutput("SmartIntake/TimedOut", timedOut);
    Logger.recordOutput("SmartIntake/Interrupted", interrupted);
    Logger.recordOutput(
        "SmartIntake/TotalIntakeTime", detectionTime >= 0 ? timer.get() - detectionTime : -1);

    // In simulation mode, when outtaking, make sure the game piece is marked as gone
    if (goal == ClawMode.OUTTAKE && Constants.currentMode == Constants.Mode.SIM) {
      beamBreak.setGamePiece(false);
      Logger.recordOutput("SmartIntake/SimOutake", "Game piece released in simulation");
    }

    if (goal == ClawMode.OUTTAKE && !interrupted && !timedOut) {
      // Only add delay if we're ending because the beam break detected the piece is gone
      // and we didn't already time out
      new SequentialCommandGroup(new WaitCommand(outtakeDelay), intake.stopCommand()).schedule();
    } else {
      intake.stopCommand().schedule();
    }
  }
}
