// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class ReefDrive extends Command {
  private final Drive drivetrain;
  private final Side side;
  private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

  // Center faces of the reef scoring positions (hexagon)
  private static final Pose2d[] reefFaces = {
    // Face 1 (0 degrees)
    new Pose2d(
        Units.inchesToMeters(209.489 + 18.885),
        Units.inchesToMeters(158.500),
        Rotation2d.fromDegrees(0)),
    // Face 2 (60 degrees)
    new Pose2d(
        Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
    // Face 3 (120 degrees)
    new Pose2d(
        Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
    // Face 4 (180 degrees)
    new Pose2d(
        Units.inchesToMeters(144.003 - 18.885),
        Units.inchesToMeters(158.500),
        Rotation2d.fromDegrees(180)),
    // Face 5 (-120 degrees)
    new Pose2d(
        Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120)),
    // Face 6 (-60 degrees)
    new Pose2d(
        Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60))
  };

  private static final double SIDE_OFFSET = Units.inchesToMeters(6.469);
  private Pose2d closestReefPose;
  private double maxSpeed = 4.0; // meters per second

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  public ReefDrive(Drive drivetrain, Side side) {
    this.drivetrain = drivetrain;
    this.side = side;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    closestReefPose = drivetrain.findClosestPose(reefFaces);
    SmartDashboard.putNumber("Reef Face Angle", closestReefPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    // Get joystick inputs (you'll need to pass these in or get them from your Controls class)
    double xSpeed = -getDriverXInput(); // Negate because forward is negative
    double ySpeed = -getDriverYInput(); // Negate because left is negative

    // Scale inputs to max speed
    xSpeed *= maxSpeed;
    ySpeed *= maxSpeed;

    // Get the reef face angle
    double faceAngle = closestReefPose.getRotation().getRadians();

    // Calculate the locked movement based on the face angle
    double[] lockedSpeeds = calculateLockedSpeeds(xSpeed, ySpeed, faceAngle);

    // Apply the locked speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            lockedSpeeds[0], lockedSpeeds[1], 0.0 // No rotation allowed during reef alignment
            );

    // Convert to robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getRotation());

    // Apply the speeds to the drivetrain
    drivetrain.setControl(
        request
            .withVelocityX(robotRelativeSpeeds.vxMetersPerSecond)
            .withVelocityY(robotRelativeSpeeds.vyMetersPerSecond)
            .withRotationalRate(robotRelativeSpeeds.omegaRadiansPerSecond));

    // Log values for debugging
    SmartDashboard.putNumber("Locked X Speed", lockedSpeeds[0]);
    SmartDashboard.putNumber("Locked Y Speed", lockedSpeeds[1]);
  }

  private double[] calculateLockedSpeeds(double xSpeed, double ySpeed, double faceAngle) {
    // For cardinal directions (0, 90, 180, 270), we can simply lock one axis
    if (isCardinalAngle(faceAngle)) {
      if (Math.abs(Math.cos(faceAngle)) > Math.abs(Math.sin(faceAngle))) {
        // Lock Y movement for faces closer to 0 or 180 degrees
        return new double[] {xSpeed, 0.0};
      } else {
        // Lock X movement for faces closer to 90 or 270 degrees
        return new double[] {0.0, ySpeed};
      }
    }

    // For diagonal faces, project the movement onto the allowed direction
    double faceVector_x = Math.cos(faceAngle);
    double faceVector_y = Math.sin(faceAngle);

    // Calculate dot product of input vector with face vector
    double dotProduct = xSpeed * faceVector_x + ySpeed * faceVector_y;

    // Project the input onto the face vector
    double projectedSpeed_x = dotProduct * faceVector_x;
    double projectedSpeed_y = dotProduct * faceVector_y;

    return new double[] {projectedSpeed_x, projectedSpeed_y};
  }

  private boolean isCardinalAngle(double angle) {
    // Check if the angle is within a small threshold of 0, 90, 180, or 270 degrees
    double degrees = Math.toDegrees(angle) % 360;
    if (degrees < 0) degrees += 360;

    double threshold = 5.0; // degrees
    return Math.abs(degrees % 90) < threshold;
  }

  // These methods should be replaced with your actual driver input methods
  private double getDriverXInput() {
    // Replace with your actual driver X input (-1 to 1)
    return 0.0;
  }

  private double getDriverYInput() {
    // Replace with your actual driver Y input (-1 to 1)
    return 0.0;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return false; // Run until cancelled
  }
}
