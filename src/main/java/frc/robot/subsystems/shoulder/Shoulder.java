package frc.robot.subsystems.shoulder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.shoulder.ShoulderIOInputsAutoLogged;
import frc.robot.utils.SignalProcessor;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.Future;

/**
 * The Shoulder subsystem controls a quad-motor shoulder mechanism for game piece manipulation. It
 * supports multiple positions for different game actions and provides both open-loop and
 * closed-loop control options.
 */
public class Shoulder extends SubsystemBase {
  public static final double FORWARD_SOFT_LIMIT_DEGREES = 70.0;
  public static final double REVERSE_SOFT_LIMIT_DEGREES = 0.0;
  private boolean isZeroed = false;
  private boolean zeroCompleted = false;
  private double zeroTimeStamp = Double.NaN;
  private Timer zeroTimer = new Timer();
  private static final double ZERO_PERCENTAGE = -0.05;
  private static final double ZERO_VELOCITY_TIME_PERIOD = 0.25;
  private static final double SHOULDER_ZERO_VELOCITY_THRESHOLD_DEGREES_PER_SECOND = 0.5;
  private static final double SHOULDER_HOMING_PITCH = -1.9;
  private static final double ACCEPTABLE_PITCH_ERROR_DEGREES = 1.0;
  private double setpointDegrees = 0.0;

  // Filter parameters for signal processing
  private static final double SHOULDER_POSITION_FILTER_ALPHA = 0.8;
  private static final double SHOULDER_VELOCITY_FILTER_ALPHA = 0.7;
  private double filteredPosition = 0.0;
  private double filteredVelocity = 0.0;
  
  // Future objects for async processing results
  private Future<?> positionProcessingFuture;
  private Future<?> velocityProcessingFuture;

  private boolean isInitialized = false;

  public enum WantedState {
    IDLE,
    HOME,
    MOVE_TO_TARGET,
    FLOOR_INTAKE,
    SOURCE_INTAKE,
    CLIMB_UP,
    CLIMB_DOWN
  }

  public enum SystemState {
    IS_IDLE,
    HOMING,
    MOVING,
    FLOOR_INTAKING,
    SOURCE_INTAKING,
    CLIMB_READYING,
    CLIMBING
  }

  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;

  private ShoulderIO io;
  private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();

  // Alerts for electronics status
  private Alert flMotorDisconnected =
      new Alert("FL Shoulder motor disconnected!", Alert.AlertType.kWarning);
  private Alert frMotorDisconnected =
      new Alert("FR Shoulder motor disconnected!", Alert.AlertType.kWarning);
  private Alert blMotorDisconnected =
      new Alert("BL Shoulder motor disconnected!", Alert.AlertType.kWarning);
  private Alert brMotorDisconnected =
      new Alert("BR Shoulder motor disconnected!", Alert.AlertType.kWarning);
  private Alert encoderDisconnected = new Alert("Shoulder encoder disconnected!", AlertType.kError);
  
  /**
   * Creates a new Shoulder subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the shoulder
   */
  public Shoulder(ShoulderIO io) {
    this.io = io;
    zeroTimer.reset();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Shoulder", inputs);

    // Process position and velocity data asynchronously with filtering
    processSignals();

    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Shoulder/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IS_IDLE;
    }

    if (DriverStation.isTeleop()) {
      // if (!isClimberStowed.getAsBoolean()) {
      //     shoulderIO.setSetpointInDegrees(0.0);
      //     ampBarIO.setSetpointInDegrees(0.0);
      // }
    }
    switch (systemState) {
      case HOMING:
        handleHoming();
        break;
      case MOVING:
        if (isZeroed) {
          io.setSetpointInDegrees(setpointDegrees);
        } else {
          wantedState = WantedState.HOME;
          handleHoming();
        }
        break;
      case FLOOR_INTAKING:
        if (isZeroed) {
          io.setSetpointInDegrees(setpointDegrees);
        } else {
          wantedState = WantedState.HOME;
          handleHoming();
        }
        break;
      case CLIMBING:
        if (isZeroed) {
          io.setPercentage(0.0);
        } else {
          wantedState = WantedState.HOME;
          handleHoming();
        }
        break;
      case IS_IDLE:
      default:
        io.setPercentage(0.0);
    }

    // Write outputs
    Logger.recordOutput("Shoulder/WantedState", wantedState.toString());
    Logger.recordOutput("Shoulder/PitchSetpointDegrees", setpointDegrees);
    Logger.recordOutput("Shoulder/IsZeroed", isZeroed);
    Logger.recordOutput("Shoulder/FilteredPosition", filteredPosition);
    Logger.recordOutput("Shoulder/FilteredVelocity", filteredVelocity);

    // Alerts
    flMotorDisconnected.set(!inputs.flConnected);
    frMotorDisconnected.set(!inputs.frConnected);
    blMotorDisconnected.set(!inputs.blConnected);
    brMotorDisconnected.set(!inputs.brConnected);
    encoderDisconnected.set(!inputs.encoderConnected);
  }

  /**
   * Process position and velocity signals asynchronously with filtering.
   */
  private void processSignals() {
    // Process position data asynchronously
    positionProcessingFuture = SignalProcessor.processAsync(
        () -> inputs.shoulderPositionDegrees,
        rawPosition -> {
            filteredPosition = SignalProcessor.lowPassFilter(
                rawPosition, 
                filteredPosition, 
                SHOULDER_POSITION_FILTER_ALPHA);
            Logger.recordOutput("Shoulder/FilteredPosition", filteredPosition);
        },
        "Shoulder/Position");
    
    // Process velocity data asynchronously
    velocityProcessingFuture = SignalProcessor.processAsync(
        () -> inputs.shoulderVelocityDegrees,
        rawVelocity -> {
            filteredVelocity = SignalProcessor.lowPassFilter(
                rawVelocity, 
                filteredVelocity, 
                SHOULDER_VELOCITY_FILTER_ALPHA);
            Logger.recordOutput("Shoulder/FilteredVelocity", filteredVelocity);
        },
        "Shoulder/Velocity");
  }

  private SystemState handleStateTransitions() {
    switch (wantedState) {
      case HOME:
        zeroCompleted = false;
        if (!DriverStation.isDisabled()) {
          if (filteredVelocity < SHOULDER_ZERO_VELOCITY_THRESHOLD_DEGREES_PER_SECOND) {
            if (!Double.isFinite(zeroTimeStamp)) {
              zeroTimeStamp = Timer.getFPGATimestamp();
              return SystemState.HOMING;
            } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
              io.setHomingPosition(SHOULDER_HOMING_PITCH);
              isZeroed = true;
              zeroCompleted = true;
              wantedState = WantedState.IDLE;
              zeroTimeStamp = Double.NaN;
              return SystemState.IS_IDLE;
            } else {
              return SystemState.HOMING;
            }
          }
          zeroTimeStamp = Double.NaN;
          return SystemState.HOMING;
        } else {
          return SystemState.HOMING;
        }
      case MOVE_TO_TARGET:
        return SystemState.MOVING;
      case FLOOR_INTAKE:
        return SystemState.FLOOR_INTAKING;
      case CLIMB_UP:
        return SystemState.CLIMB_READYING;
      case CLIMB_DOWN:
        return SystemState.CLIMBING;
      case IDLE:
      default:
        return SystemState.IS_IDLE;
    }
  }

  private void handleHoming() {
    if (isZeroed) {
      io.disableSoftLimits();
      isZeroed = false;
    }
    io.setPercentage(ZERO_PERCENTAGE);
  }

  public Command defaultCommand() {
    // implicitly require 'this'
    // return a command that initializes once and then does nothing because the periodic method
    // handles everything
    return this.run(
        () -> {
          if (!isInitialized) {
            setWantedState(WantedState.IDLE);
            isInitialized = true;
          }
        });
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, Angle angleDegrees) {
    this.wantedState = wantedState;
    setTargetPitchDegrees(angleDegrees);
  }

  public void setTargetPitchDegrees(Angle angleDegrees) {
    double clampedDegrees =
        MathUtil.clamp(angleDegrees.abs(Degrees), REVERSE_SOFT_LIMIT_DEGREES, FORWARD_SOFT_LIMIT_DEGREES);
    setpointDegrees = clampedDegrees;
  }

  public boolean zeroCompleted() {
    return zeroCompleted;
  }

  public void setZeroCompleted(boolean zeroCompleted) {
    this.zeroCompleted = zeroCompleted;
  }

  public boolean shoulderAtSetpoint() {
    return MathUtil.isNear(
        setpointDegrees, filteredPosition, ACCEPTABLE_PITCH_ERROR_DEGREES);
  }

  public void setAngle(Angle angleDegrees) {
    io.setHomingPosition(angleDegrees.abs(Degrees));
    wantedState = WantedState.IDLE;
    systemState = SystemState.IS_IDLE;
    isZeroed = true;
    zeroCompleted = true;
    zeroTimeStamp = Double.NaN;
  }

  public double getCurrentPosition() {
    return filteredPosition;
  }
  
  /**
   * Cancels any ongoing signal processing operations.
   * Call this when the robot is disabled or the subsystem is no longer needed.
   */
  public void cancelProcessing() {
    if (positionProcessingFuture != null && !positionProcessingFuture.isDone()) {
      positionProcessingFuture.cancel(true);
    }
    if (velocityProcessingFuture != null && !velocityProcessingFuture.isDone()) {
      velocityProcessingFuture.cancel(true);
    }
  }
}