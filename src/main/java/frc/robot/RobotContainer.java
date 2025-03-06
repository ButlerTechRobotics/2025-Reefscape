// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SmartArm;
import frc.robot.commands.SmartIntake;
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
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.beambreak.BeamBreakIO;
import frc.robot.subsystems.beambreak.BeamBreakIOReal;
import frc.robot.subsystems.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ClawMode;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.onboardbuttons.OnBoardButtons;
import frc.robot.subsystems.onboardbuttons.OnBoardButtonsIO;
import frc.robot.subsystems.onboardbuttons.OnBoardButtonsIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.DisabledInstantCommand;
import frc.robot.utils.DriverController;
import frc.robot.utils.OperatorController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final DriverController driver = new DriverController(0, TunableControllerType.LINEAR);
  private final OperatorController operator =
      new OperatorController(1, TunableControllerType.LINEAR);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  public final Intake intake;
  public final BeamBreak beamBreak;
  public final Arm arm;
  public final OnBoardButtons onBoardButtons;

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
                        Units.inchesToMeters(13),
                        Units.inchesToMeters(11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(30))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVision(
                "FR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(13),
                        Units.inchesToMeters(-11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(330))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVision(
                "BL-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-13),
                        Units.inchesToMeters(11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(210))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVision(
                "BR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-13),
                        Units.inchesToMeters(-11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(150))),
                drivetrain::getVisionParameters));

        intake = new Intake(new IntakeIOCTRE());
        beamBreak = new BeamBreak(new BeamBreakIOReal(0));
        extension = new Extension(new ExtensionIOCTRE());
        shoulder = new Shoulder(new ShoulderIOCTRE());
        wrist = new Wrist(new WristIOCTRE());
        onBoardButtons = new OnBoardButtons(new OnBoardButtonsIOReal(3, 4));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVisionSIM(
                "FL-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(13),
                        Units.inchesToMeters(11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(30))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "FR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(13),
                        Units.inchesToMeters(-11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(330))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "BL-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-13),
                        Units.inchesToMeters(11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(210))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "BR-Camera",
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-13),
                        Units.inchesToMeters(-11.5),
                        Units.inchesToMeters(9.0)),
                    new Rotation3d(0, Math.toRadians(10), Math.toRadians(150))),
                drivetrain::getVisionParameters));

        intake = new Intake(new IntakeIOSIM());
        beamBreak = new BeamBreak(new BeamBreakIOSim(0));
        extension = new Extension(new ExtensionIOSIM());
        shoulder = new Shoulder(new ShoulderIOSIM());
        wrist = new Wrist(new WristIOSIM());
        onBoardButtons = new OnBoardButtons(new OnBoardButtonsIO() {});
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

        intake = new Intake(new IntakeIO() {});
        beamBreak = new BeamBreak(new BeamBreakIO() {});
        extension = new Extension(new ExtensionIO() {});
        shoulder = new Shoulder(new ShoulderIO() {});
        wrist = new Wrist(new WristIO() {});
        onBoardButtons = new OnBoardButtons(new OnBoardButtonsIO() {});
        break;
    }

    arm = new Arm(shoulder, extension, wrist);

    // Set up the named commands
    NamedCommands.registerCommand("Stow", arm.setGoalCommand(Arm.Goal.STOW));
    NamedCommands.registerCommand("Standby", arm.setGoalCommand(Arm.Goal.STANDBY));
    NamedCommands.registerCommand(
        "Coral_Floor_Intake", arm.setGoalCommand(Arm.Goal.CORAL_FLOOR_INTAKE));
    NamedCommands.registerCommand(
        "Coral_Station_Intake", arm.setGoalCommand(Arm.Goal.CORAL_STATION_INTAKE));
    NamedCommands.registerCommand("Coral_L4Back", arm.setGoalCommand(Arm.Goal.CORAL_L4BACK));
    NamedCommands.registerCommand(
        "Score", new SmartIntake(intake, beamBreak, ClawMode.OUTTAKE, 0.25));
    NamedCommands.registerCommand(
        "SIMGamePiecePickup", new InstantCommand(() -> beamBreak.setGamePiece(true)));

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

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drivetrain
                    .setpointGen
                    .withVelocityX(MaxSpeed.times(-driver.getLeftY()))
                    .withVelocityY(MaxSpeed.times(-driver.getLeftX()))
                    .withRotationalRate(Constants.MaxAngularRate.times(-driver.getRightX()))
                    .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver.resetHeading().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    // joystick
    //     .leftTrigger()
    //     .and(joystick.a())
    //     .whileTrue(new ReefDrive(drivetrain, ReefDrive.Side.LEFT));

    driver
        .floorIntake()
        .whileTrue(new SmartIntake(intake, beamBreak, Intake.ClawMode.STATION_INTAKE));
    driver.outtake().whileTrue(new SmartIntake(intake, beamBreak, Intake.ClawMode.OUTTAKE, 1.0));

    driver.povUp().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L3BACK));
    driver.povLeft().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L2BACK));
    driver.povDown().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L1));
    driver.povRight().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_STATION_INTAKE));

    driver.leftTrigger().onTrue(new SmartArm(arm, SmartArm.Goal.CLIMB));
    driver.rightTrigger().onTrue(new SmartArm(arm, SmartArm.Goal.STOW));

    driver.a().onTrue(Commands.runOnce(() -> arm.getShoulder().setBrakeMode(false)));
    driver.b().onTrue(Commands.runOnce(() -> arm.getShoulder().setBrakeMode(true)));

    // joystick.povLeft().whileTrue(AutoScore.scoreAtL4(drivetrain, arm, ReefDrive.Side.RIGHT));

    // Button bindings for the physical buttons on the robot
    new Trigger(onBoardButtons::getHomeButtonPressed)
        .onTrue(
            new DisabledInstantCommand(
                () -> {
                  if (DriverStation.isDisabled()) {
                    arm.getShoulder().setZero();
                    // arm.getExtension().setZero();
                    // arm.getWrist().setZero();
                  }
                }));

    new Trigger(onBoardButtons::getBrakeButtonPressed)
        .onTrue(
            new DisabledInstantCommand(
                () -> {
                  if (DriverStation.isDisabled()) {
                    arm.getShoulder().toggleBrakeMode();
                  }
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
