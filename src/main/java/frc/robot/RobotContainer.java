// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefDrive;
import frc.robot.commands.SmartArm;
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.leds.LEDs;
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
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.15))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.15)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public final Intake intake;
  public final Arm arm;
  public final OnBoardButtons onBoardButtons;
  private final LEDs leds;

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
        extension = new Extension(new ExtensionIO() {});
        shoulder = new Shoulder(new ShoulderIO() {});
        wrist = new Wrist(new WristIO() {});
        onBoardButtons = new OnBoardButtons(new OnBoardButtonsIO() {});
        break;
    }

    arm = new Arm(shoulder, extension, wrist, intake);
    leds = new LEDs();

    // Set up the named commands
    NamedCommands.registerCommand(
        "Score", Commands.waitUntil(() -> arm.isAtTarget()).andThen(intake.AUTO_SHOOT()));

    Command coralL4BackLeft = arm.setGoalCommand(Arm.Goal.CORAL_L4BACK);
    Command coralL4BackRight = arm.setGoalCommand(Arm.Goal.CORAL_L4BACK);
    Command shuffleCoralToBack = intake.shuffleCoralToBack();
    Command standby = arm.setGoalCommand(Arm.Goal.STANDBY).withDeadline(Commands.waitSeconds(0.5));
    Command coralFloorIntake =
        arm.setGoalCommand(Arm.Goal.CORAL_FLOOR_INTAKE)
            .withDeadline(Commands.waitSeconds(0.5))
            .andThen(intake.AUTO_INTAKE().andThen(standby));

    NamedCommands.registerCommand(
        "Align_Left",
        Commands.parallel(new ReefDrive(drivetrain, ReefDrive.Side.LEFT), coralL4BackLeft)
            .withDeadline(Commands.waitSeconds(2))
            .andThen(intake.AUTO_SHOOT())
            .withTimeout(5));

    NamedCommands.registerCommand(
        "Align_Right",
        Commands.parallel(new ReefDrive(drivetrain, ReefDrive.Side.RIGHT), coralL4BackRight)
            .withDeadline(Commands.waitSeconds(2))
            .andThen(intake.AUTO_SHOOT())
            .withTimeout(5));
    NamedCommands.registerCommand("Standby", standby);
    NamedCommands.registerCommand("Coral_Floor_Intake", coralFloorIntake);
    NamedCommands.registerCommand("Shuffle", shuffleCoralToBack);

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
    // Set up the default drive command with pre-allocated request object
    configureDriveCommand();

    // Create shared triggers to avoid duplicate condition objects
    configureSharedTriggers();

    // Set up manual control buttons
    configureManualButtons();
  }
  /** Configure the default drive command using pre-allocated request objects. */
  private void configureDriveCommand() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -driver.customLeft().getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -driver.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -driver
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)
  }

  /** Create shared trigger objects for commonly used conditions. */
  private void configureSharedTriggers() {
    // // Define commonly used triggers once to avoid duplicate object creation
    // Trigger isScoringPosition =
    //     new Trigger(
    //         () ->
    //             (arm.getGoal() == Goal.CORAL_L1
    //                 || arm.getGoal() == Goal.CORAL_L1BACK
    //                 || arm.getGoal() == Goal.CORAL_L2
    //                 || arm.getGoal() == Goal.CORAL_L2BACK
    //                 || arm.getGoal() == Goal.CORAL_L3
    //                 || arm.getGoal() == Goal.CORAL_L3BACK
    //                 || arm.getGoal() == Goal.CORAL_L4BACK));

    // // Use slower driving while in scoring position
    // isScoringPosition.whileTrue(
    //     drivetrain.applyRequest(
    //         () ->
    //             drive
    //                 .withVelocityX(MetersPerSecond.of(1).times(-driver.customLeft().getY()))
    //                 .withVelocityY(MetersPerSecond.of(1).times(-driver.customLeft().getX()))
    //                 .withRotationalRate(
    //                     Constants.MaxAngularRate.times(-driver.customRight().getX()))));

    // Create a trigger that detects when we have a game piece while in floor intake position
    Trigger gamePickedUpInFloorIntake =
        new Trigger(
            () -> intake.hasBackGamePiece() && (arm.getGoal() == Arm.Goal.CORAL_FLOOR_INTAKE));

    // When this trigger becomes active, move to standby position AND shuffle the coral to back
    gamePickedUpInFloorIntake.onTrue(
        Commands.sequence(
            // Log that we detected a game piece
            Commands.runOnce(
                () -> System.out.println("Game piece detected in floor intake - processing")),

            // Stop the intake motors
            intake.STOP(),

            // Then move to standby position
            Commands.runOnce(() -> System.out.println("Moving arm to standby")),
            new SmartArm(arm, SmartArm.Goal.STANDBY),

            // First shuffle the coral to the back position
            Commands.runOnce(() -> System.out.println("Shuffling coral to back")),
            intake.shuffleCoralToBack()));

    Trigger gamePickedUpSetLeds = new Trigger(() -> intake.hasBackGamePiece());

    Trigger gameScored = new Trigger(() -> !intake.hasGamePiece());

    gamePickedUpSetLeds.onTrue(Commands.runOnce(() -> leds.setHasGamePiece(true)));

    gameScored.onTrue(Commands.runOnce(() -> leds.setHasGamePiece(false)));
  }

  private void configureManualButtons() {
    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver.resetHeading().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    driver
        .leftStick()
        .whileTrue(
            new ReefDrive(drivetrain, ReefDrive.Side.LEFT)
                .alongWith(
                    Commands.startEnd(
                        () -> leds.setIsAutoAligning(true), () -> leds.setIsAutoAligning(false))));

    driver
        .rightStick()
        .whileTrue(
            new ReefDrive(drivetrain, ReefDrive.Side.RIGHT)
                .alongWith(
                    Commands.startEnd(
                        () -> leds.setIsAutoAligning(true), () -> leds.setIsAutoAligning(false))));

    // Driver Button Bindings
    driver.intake().whileTrue(intake.intakeCoralToBack()).onFalse(intake.STOP());
    driver
        .shoot()
        .onTrue(
            Commands.either(
                intake.scoreCoralFromBack(), intake.scoreCoralFromFront(), arm::isScoringFront));
    driver.povUp().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_STATION_INTAKE));
    driver.povLeft().onTrue(arm.coralPreIntakeToFloorIntake());
    driver.povRight().onTrue(new SmartArm(arm, SmartArm.Goal.STANDBY));
    driver.povDown().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_FLOOR_INTAKE));

    driver.a().onTrue(new SmartArm(arm, SmartArm.Goal.CLIMB_DOWN));
    driver.y().onTrue(new SmartArm(arm, SmartArm.Goal.CLIMB));
    driver.b().onTrue(intake.shuffleCoralToBack());

    // Operator Button Bindings
    operator.coralL1().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L1));
    operator.coralL2().onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L2BACK));
    operator
        .coralL3()
        .onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L3BACK))
        .onTrue(intake.shuffleCoralToBack());
    operator
        .coralL4()
        .onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L4BACK))
        .onTrue(intake.shuffleCoralToBack());

    operator
        .coralL2()
        .and(operator.frontModifier())
        .onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L2));
    operator
        .coralL3()
        .and(operator.frontModifier())
        .onTrue(new SmartArm(arm, SmartArm.Goal.CORAL_L3))
        .onTrue(intake.shuffleCoralToFront());
    operator.algaeL1().onTrue(new SmartArm(arm, SmartArm.Goal.ALGAE_L1));
    operator.algaeL2().onTrue(new SmartArm(arm, SmartArm.Goal.ALGAE_L2));
    operator.algaeScore().onTrue(new SmartArm(arm, SmartArm.Goal.ALGAE_SCORE));
    operator.algaeFloorIntake().onTrue(new SmartArm(arm, SmartArm.Goal.ALGAE_FLOOR_INTAKE));
    operator
        .axisGreaterThan(1, 0.5)
        .whileTrue(
            Commands.runEnd(
                () -> arm.getShoulder().setVoltage(Volts.of(1)),
                () -> arm.getShoulder().setVoltage(Volts.of(0)),
                arm.getShoulder()));
    operator
        .axisLessThan(1, -0.5)
        .whileTrue(
            Commands.runEnd(
                () -> arm.getShoulder().setVoltage(Volts.of(-1)),
                () -> arm.getShoulder().setVoltage(Volts.of(0)),
                arm.getShoulder()));
    operator
        .axisGreaterThan(5, 0.5)
        .whileTrue(
            Commands.runEnd(
                () -> arm.getWrist().setVoltage(Volts.of(1)),
                () -> arm.getWrist().setVoltage(Volts.of(0)),
                arm.getWrist()));
    operator
        .axisLessThan(5, -0.5)
        .whileTrue(
            Commands.runEnd(
                () -> arm.getWrist().setVoltage(Volts.of(-1)),
                () -> arm.getWrist().setVoltage(Volts.of(0)),
                arm.getWrist()));

    // // Button bindings for the physical buttons on the robot
    new Trigger(onBoardButtons::getHomeButtonPressed)
        .onTrue(
            new DisabledInstantCommand(
                () -> {
                  if (DriverStation.isDisabled()) {
                    arm.getWrist().setZero();
                  }
                }));

    new Trigger(onBoardButtons::getBrakeButtonPressed)
        .onTrue(
            new DisabledInstantCommand(
                () -> {
                  if (DriverStation.isDisabled()) {
                    arm.getWrist().toggleBrakeMode();
                  }
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
