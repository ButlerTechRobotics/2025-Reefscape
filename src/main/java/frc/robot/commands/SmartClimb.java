package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Translation2d;

public class SmartClimb extends Command {
    private final Drive drivetrain;
    private final Arm arm;
    private ClimbState currentState;
    
    private enum ClimbState {
        INIT,           // Initial state
        PREPARE_ARM,    // Move arm to climb position
        DRIVE_IN,       // Drive into chain
        LIFT,          // Lift robot
        DONE           // Climbing complete
    }
    
    public SmartClimb(Drive drivetrain, Arm arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        addRequirements(drivetrain, arm);
    }
    
    @Override
    public void initialize() {
        currentState = ClimbState.INIT;
    }
    
    @Override
    public void execute() {
        switch (currentState) {
            case INIT:
                // Start sequence
                currentState = ClimbState.PREPARE_ARM;
                break;
                
            case PREPARE_ARM:
                // Set arm to climb position
                arm.setGoalCommand(Arm.Goal.CLIMB).schedule();
                if (arm.isAtTarget()) {
                    currentState = ClimbState.DRIVE_IN;
                }
                break;
                
            case DRIVE_IN:
                // Drive forward slowly until contact
                drivetrain.drive(new Translation2d(0.2, 0), 0, true, true);
                // TODO: Add contact detection logic
                if (detectChainContact()) {
                    currentState = ClimbState.LIFT;
                }
                break;
                
            case LIFT:
                // Lift robot using arm
                arm.setGoalCommand(Arm.Goal.STOW).schedule();
                if (arm.isAtTarget()) {
                    currentState = ClimbState.DONE;
                }
                break;
                
            case DONE:
                // Climbing complete
                break;
        }
    }
    
    private boolean detectCageContact() {
        // TODO: Implement cage contact detection
        // Could use current spike, limit switch, or distance sensor
        return false;
    }
    
    @Override
    public boolean isFinished() {
        return currentState == ClimbState.DONE;
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        if (interrupted) {
            arm.setGoalCommand(Arm.Goal.STOW).schedule();
        }
    }
}