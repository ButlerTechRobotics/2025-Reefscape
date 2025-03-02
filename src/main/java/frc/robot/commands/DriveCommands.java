// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants.Reef;
import frc.robot.utils.GeomUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import org.littletonrobotics.junction.Logger;

public class DriveCommands extends Command {

  private static PIDController translationController = new PIDController(5.0, 0, 0);
  private static PIDController rotationController = new PIDController(8, 0, 0);

  static {
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(0.1);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    SwerveRequest.RobotCentric req =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withVelocityX(0)
            .withVelocityY(0);

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(6);
                  drive.setControl(req.withRotationalRate(speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getDrivePositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      Angle[] positions = drive.getDrivePositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < Constants.PP_CONFIG.numModules; i++) {
                        wheelDelta +=
                            Math.abs(
                                    positions[i].minus(state.positions[i]).baseUnitMagnitude()
                                        / Constants.SWERVE_MODULE_CONSTANTS.DriveMotorGearRatio)
                                / Constants.PP_CONFIG.numModules;
                      }
                      double wheelRadius =
                          (state.gyroDelta * Constants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                      System.out.println(
                          "\tDrive base radius: "
                              + formatter.format(Constants.DRIVE_BASE_RADIUS)
                              + " meters");
                    })));
  }

  public static void driveToPointMA(Pose2d target, Drive drive) {
    driveToPointMA(target, drive, Constants.robotScoringOffset);
  }

  public static void driveToPointMA(Pose2d target, Drive drive, Distance offset) {
    Pose2d robotPose = drive.getPose();

    // Calculate approach path using the oriented target
    // Get the properly oriented target based on which side of the robot is closer to target
    // rotation
    Pose2d orientedTarget = getOptimalOrientedTarget(robotPose, target);
    Pose2d newTarget = getDriveTarget(robotPose, orientedTarget, offset);

    driveToPoint(orientedTarget, drive);
  }

  /**
   * Determines if the robot's back side is closer to the target orientation.
   *
   * @param robotPose Current robot pose
   * @param targetPose Target pose
   * @return true if the back side of the robot is closer to the target orientation
   */
  public static boolean isBackSideCloser(Pose2d robotPose, Pose2d targetPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d targetRotation = targetPose.getRotation();

    // Calculate angular difference if using front of robot
    double frontDifference = Math.abs(currentRotation.minus(targetRotation).getRadians());

    // Calculate angular difference if using back of robot (flipped 180 degrees)
    Rotation2d flippedRotation = currentRotation.plus(Rotation2d.fromDegrees(180));
    double backDifference = Math.abs(flippedRotation.minus(targetRotation).getRadians());

    // Return true if back side is closer
    return backDifference < frontDifference;
  }

  /**
   * Gets the optimal target pose based on whether front or back of robot should face the target.
   *
   * @param robotPose Current robot pose
   * @param targetPose Target pose
   * @return Modified target pose oriented for optimal approach
   */
  public static Pose2d getOptimalOrientedTarget(Pose2d robotPose, Pose2d targetPose) {
    // Use the helper method to check if back side is closer
    boolean useBackSide = isBackSideCloser(robotPose, targetPose);

    // If back is closer, flip the target rotation
    if (useBackSide) {
      return GeomUtil.flipRotation(targetPose);
    }

    // Otherwise, use the original target
    return targetPose;
  }

  /** Get drive target. */
  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal, Distance robotOffset) {

    // Final line up
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());

    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength.in(Meters) * 2))
                + ((xDistance - 0.3) / (Reef.faceLength.in(Meters) * 3)),
            0.0,
            1.0);
    double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength.in(Meters), 0.0, 1.0);

    Pose2d goalPose =
        goal.transformBy(
            GeomUtil.toTransform2d(
                -shiftXT * Constants.maxDistanceReefLineup.in(Meters),
                Math.copySign(
                    shiftYT * Constants.maxDistanceReefLineup.in(Meters) * 0.8, offset.getY())));

    Logger.recordOutput("Drive/TargetPose", goalPose);
    
    return goalPose;
  }

  public static void driveToPoint(Pose2d target, Drive drive) {
    driveToPoint(target, drive, Constants.robotScoringOffset);
  }

  public static void driveToPoint(Pose2d target, Drive drive, Distance offset) {
    Pose2d current = drive.getPose();
    double pidX = translationController.calculate(current.getX(), target.getX());
    double pidY = translationController.calculate(current.getY(), target.getY());
    double pidRot =
        rotationController.calculate(
            drive.getRotation().getRotations(), target.getRotation().getRotations());
    ChassisSpeeds speeds = new ChassisSpeeds(pidX, pidY, Rotations.of(pidRot).in(Radians));

    SwerveSetpointGen setpointGenerator = drive.getSetpointGenerator();

    setpointGenerator
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond);
    drive.setControl(setpointGenerator);
  }

  private static class WheelRadiusCharacterizationState {
    Angle[] positions = new Angle[Constants.PP_CONFIG.numModules];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
