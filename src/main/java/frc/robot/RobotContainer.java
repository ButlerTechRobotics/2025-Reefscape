// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoArm;
import frc.robot.commands.AutoClaw;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.extension.Extension;
import frc.robot.subsystems.arm.extension.ExtensionIO;
import frc.robot.subsystems.arm.extension.ExtensionIOCTRE;
import frc.robot.subsystems.arm.extension.ExtensionIOSIM;
import frc.robot.subsystems.arm.shoulder.Shoulder;
import frc.robot.subsystems.arm.shoulder.ShoulderIO;
import frc.robot.subsystems.arm.shoulder.ShoulderIOCTRE;
import frc.robot.subsystems.arm.shoulder.ShoulderIOSIM;
import frc.robot.subsystems.arm.wrist.Wrist;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIOCTRE;
import frc.robot.subsystems.arm.wrist.WristIOSIM;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIO;
import frc.robot.subsystems.claw.ClawIOCTRE;
import frc.robot.subsystems.claw.ClawIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController joystick =
      new TunableController(0).withControllerType(TunableControllerType.CUBIC);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  public final Claw claw;
  public final Arm arm;

  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    // Declare component subsystems (not visible outside constructor)
    Extension extension = null;
    Shoulder shoulder = null;
    Wrist wrist = null;

    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVision(
                "FL-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(10.5),
                        Units.inchesToMeters(10.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(30))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVision(
                "FR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(10.5),
                        Units.inchesToMeters(-10.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(-30))),
                drivetrain::getVisionParameters));

        claw = new Claw(new ClawIOCTRE());
        extension = new Extension(new ExtensionIOCTRE());
        shoulder = new Shoulder(new ShoulderIOCTRE());
        wrist = new Wrist(new WristIOCTRE());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVisionSIM(
                "BL-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-10.5),
                        Units.inchesToMeters(10.5),
                        Units.inchesToMeters(6.5)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(150))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "BR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-10.5),
                        Units.inchesToMeters(-10.5),
                        Units.inchesToMeters(6.5)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(210))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "FL-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(10.375),
                        Units.inchesToMeters(10.375),
                        Units.inchesToMeters(10.25)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(30))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "FR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-10.375),
                        Units.inchesToMeters(-10.375),
                        Units.inchesToMeters(10.25)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(330))),
                drivetrain::getVisionParameters));

        claw = new Claw(new ClawIOSIM());
        extension = new Extension(new ExtensionIOSIM());
        shoulder = new Shoulder(new ShoulderIOSIM());
        wrist = new Wrist(new WristIOSIM());
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});

        new Vision(
            drivetrain::addVisionData,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});

        claw = new Claw(new ClawIO() {});
        extension = new Extension(new ExtensionIO() {});
        shoulder = new Shoulder(new ShoulderIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    arm = new Arm(shoulder, extension, wrist);

    // Set up the named commands
    NamedCommands.registerCommand("L1Align", new AutoArm(arm, AutoArm.ArmMode.L1));
    NamedCommands.registerCommand("L2Align", new AutoArm(arm, AutoArm.ArmMode.L2));
    NamedCommands.registerCommand("L3Align", new AutoArm(arm, AutoArm.ArmMode.L3));
    NamedCommands.registerCommand("L4Align", new AutoArm(arm, AutoArm.ArmMode.L4));
    NamedCommands.registerCommand(
        "StationIntake", new AutoArm(arm, AutoArm.ArmMode.STATION_INTAKE));
    NamedCommands.registerCommand("Score", new AutoClaw(claw, AutoClaw.ClawMode.FLOOR_INTAKE));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drivetrain));
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -joystick
                                .customLeft()
                                .getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -joystick.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)

    // joystick.a().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));
    // joystick
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
                drivetrain.getChassisSpeeds(),
                drivetrain.getModuleStates(),
                drivetrain::getRotation)
            .withDeadband(MaxSpeed.times(0.1))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    joystick
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    setpointGen
                        .withVelocityX(
                            MaxSpeed.times(
                                -joystick.getLeftY())) // Drive forward with negative Y (forward)
                        .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
                        .withRotationalRate(Constants.MaxAngularRate.times(-joystick.getRightX()))
                        .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Custom Swerve Request that use ProfiledFieldCentricFacingAngle. Allows you to face specific
    // direction while driving
    ProfiledFieldCentricFacingAngle driveFacingAngle =
        new ProfiledFieldCentricFacingAngle(
                new TrapezoidProfile.Constraints(
                    Constants.MaxAngularRate.baseUnitMagnitude(),
                    Constants.MaxAngularRate.div(0.25).baseUnitMagnitude()))
            .withDeadband(MaxSpeed.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Set PID for ProfiledFieldCentricFacingAngle
    driveFacingAngle.HeadingController.setPID(7, 0, 0);
    // joystick
    //     .y()
    //     .whileTrue(
    //         drivetrain
    //             .runOnce(() -> driveFacingAngle.resetProfile(drivetrain.getRotation()))
    //             .andThen(
    //                 drivetrain.applyRequest(
    //                     () ->
    //                         driveFacingAngle
    //                             .withVelocityX(
    //                                 MaxSpeed.times(
    //                                     -joystick
    //                                         .getLeftY())) // Drive forward with negative Y
    // (forward)
    //                             .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
    //                             .withTargetDirection(
    //                                 new Rotation2d(
    //                                     -joystick.getRightY(), -joystick.getRightX())))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.back().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    // joystick.povDown().whileTrue(arm.setGoalCommand(Arm.Goal.STOW));
    // joystick.povLeft().whileTrue(arm.setGoalCommand(Arm.Goal.L1));
    // joystick.povRight().whileTrue(arm.setGoalCommand(Arm.Goal.L2));
    // joystick.povUp().whileTrue(arm.setGoalCommand(Arm.Goal.L3));

    // SmartDashboard.putData(
    //     "Pathfind and score L1 Left",
    //     new SmartScore(drivetrain, arm, SmartScore.Side.LEFT, SmartScore.ArmMode.L1));
    // SmartDashboard.putData(
    //     "Pathfind and score L1 Right",
    //     new SmartScore(drivetrain, arm, SmartScore.Side.RIGHT, SmartScore.ArmMode.L1));
    // joystick
    //     .leftBumper()
    //     .and(joystick.a())
    //     .whileTrue(new SmartDrive(drivetrain, SmartDrive.Side.LEFT));
    // joystick
    //     .rightBumper()
    //     .and(joystick.a())
    //     .whileTrue(new SmartDrive(drivetrain, SmartDrive.Side.RIGHT));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
