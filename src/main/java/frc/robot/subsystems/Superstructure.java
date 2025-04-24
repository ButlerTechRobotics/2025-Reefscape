package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shoulder.Shoulder;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the robot's superstructure state machine and delegates actions to individual subsystems.
 */
public class Superstructure extends SubsystemBase {
    
    // Define system states as a public enum for external use
    public enum WantedSuperState {
        IDLE,
        INTAKE_READY,
        INTAKING,
        SCORING_READY,
        SCORING,
        CLIMBING_READY,
        CLIMBING,
        STOWED
    }
    
    private WantedSuperState currentState = WantedSuperState.IDLE;
    private final Shoulder shoulder;
    // Add other subsystems as needed
    
    /**
     * Creates a new Superstructure that coordinates all robot subsystems.
     * 
     * @param shoulder The shoulder subsystem
     */
    public Superstructure(Shoulder shoulder) {
        this.shoulder = shoulder;
    }
    
    @Override
    public void periodic() {
        // Log current system state
        Logger.recordOutput("Superstructure/CurrentState", currentState.toString());
    }
    
    /**
     * Sets the current state of the superstructure and applies subsystem actions.
     * 
     * @param newState The state to transition to
     */
    public void setWantedSuperState(WantedSuperState newState) {
        if (newState == currentState) return;
        
        Logger.recordOutput("Superstructure/StateTransition", 
                            currentState + " -> " + newState);
        
        // Handle subsystem transitions based on new state
        switch (newState) {
            case IDLE:
                shoulder.setWantedState(Shoulder.WantedState.IDLE);
                break;
                
            case INTAKE_READY:
                shoulder.setWantedState(Shoulder.WantedState.FLOOR_INTAKE);
                break;
                
            case INTAKING:
                // No change to shoulder, but would activate intake motors
                break;
                
            case SCORING_READY:
                shoulder.setWantedState(Shoulder.WantedState.MOVE_TO_TARGET);
                // Set the specific angle for scoring
                shoulder.setTargetPitchDegrees(edu.wpi.first.units.Units.Degrees.of(45.0));
                break;
                
            case CLIMBING_READY:
                shoulder.setWantedState(Shoulder.WantedState.CLIMB_UP);
                break;
                
            case CLIMBING:
                shoulder.setWantedState(Shoulder.WantedState.CLIMB_DOWN);
                break;
                
            case STOWED:
                shoulder.setWantedState(Shoulder.WantedState.HOME);
                break;
                
            default:
                break;
        }
        
        currentState = newState;
    }
    
    /**
     * Creates a command to set the superstructure state.
     * 
     * @param state The desired state
     * @return Command to set the state
     */
    public Command setWantedSuperStateCommand(WantedSuperState state) {
        return Commands.runOnce(() -> setWantedSuperState(state))
                .withName("SetSuperState_" + state.toString());
    }
    
    /**
     * Gets the current state of the superstructure.
     * 
     * @return The current state
     */
    public WantedSuperState getCurrentState() {
        return currentState;
    }
    
    /**
     * Checks if the superstructure is in a specific state.
     * 
     * @param state The state to check
     * @return True if in the specified state
     */
    public boolean isInState(WantedSuperState state) {
        return currentState == state;
    }
    
    /**
     * Creates a toggle command between two states.
     * 
     * @param firstState First state
     * @param secondState Second state
     * @return Command that toggles between states
     */
    public Command toggleStateCommand(WantedSuperState firstState, WantedSuperState secondState) {
        return Commands.runOnce(() -> {
            if (currentState == firstState) {
                setWantedSuperState(secondState);
            } else {
                setWantedSuperState(firstState);
            }
        }).withName("ToggleSuperState_" + firstState + "_" + secondState);
    }
}