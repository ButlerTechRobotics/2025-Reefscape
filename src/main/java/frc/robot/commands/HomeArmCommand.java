package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.LEDs;
import org.littletonrobotics.junction.Logger;

/**
 * Zeros the arm by homing each joint in sequence.
 */
public class HomeArmCommand extends SequentialCommandGroup {
    
    private final Arm arm;
    private final LEDs leds;
    
    /**
     * Creates a command that zeros the arm subsystems in sequence.
     * @param arm The arm subsystem
     * @param leds The LED subsystem
     */
    public HomeArmCommand(Arm arm, LEDs leds) {
        this.arm = arm;
        this.leds = leds;
        
        // Create the command sequence
        addCommands(
            // Start by indicating we're zeroing
            Commands.runOnce(() -> {
                arm.setZeroed(false);
                Logger.recordOutput("Arm/Homing", "Started");
                System.out.println("Starting arm homing sequence");
            }),
            
            // // Home the wrist first
            // Commands.print("Homing wrist..."),
            // arm.getWrist().homingSequence(),
            // Commands.print("Wrist homing complete"),
            
            // // Home the extension next
            // Commands.print("Homing extension..."),
            // arm.getExtension().homingSequence(),
            // Commands.print("Extension homing complete"),
            
            // Home the shoulder last
            Commands.print("Homing shoulder..."),
            arm.getShoulder().homingSequence(),
            Commands.print("Shoulder homing complete"),
            
            // When all are complete, mark arm as zeroed
            Commands.runOnce(() -> {
                arm.setZeroed(true);
                Logger.recordOutput("Arm/Homing", "Complete");
                System.out.println("Arm homing sequence complete");
            }),
            
            // Optional: Set to stow position after homing
            arm.setGoalCommand(Arm.Goal.STOW)
        );
    }
    
    // Factory method to create the command
    public static Command create(Arm arm, LEDs leds) {
        return new HomeArmCommand(arm, leds);
    }
}