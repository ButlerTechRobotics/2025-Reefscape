package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A custom wrapper for GenericHID that provides named access to buttons on a button box.
 * This allows for more readable code when binding commands to buttons.
 * 
 * <p>Usage example:
 * <pre>
 * private final ButtonBox buttonBox = new ButtonBox(1);
 * buttonBox.l1Button().onTrue(new ExampleCommand());
 * </pre>
 */
public class ButtonBox extends GenericHID {
    
    /**
     * Constructs a new ButtonBox on the specified port.
     *
     * @param port The port index on the Driver Station (0-5) that the button box is plugged into.
     */
    public ButtonBox(int port) {
        super(port);
    }
    
    /**
     * Gets the FRONT CL1 button (button 1) as a Trigger.
     *
     * @return A Trigger representing button 1
     */
    public Trigger frontL1Button() {
        return new JoystickButton(this, 1);
    }
    
    /**
     * Gets the FRONT CL2 button (button 2) as a Trigger.
     *
     * @return A Trigger representing button 2
     */
    public Trigger frontL2Button() {
        return new JoystickButton(this, 2);
    }
    
    /**
     * Gets the FRONT CL3 button (button 3) as a Trigger.
     *
     * @return A Trigger representing button 3
     */
    public Trigger frontL3Button() {
        return new JoystickButton(this, 3);
    }
    
    /**
     * Gets the FRONT CL4 button (button 4) as a Trigger.
     *
     * @return A Trigger representing button 4
     */
    public Trigger frontL4Button() {
        return new JoystickButton(this, 4);
    }
    
    /**
     * Gets the BACK CL1 button (button 5) as a Trigger.
     *
     * @return A Trigger representing button 5
     */
    public Trigger backL1Button() {
        return new JoystickButton(this, 5);
    }
    
    /**
     * Gets the BACK CL2 button (button 6) as a Trigger.
     *
     * @return A Trigger representing button 6
     */
    public Trigger backL2Button() {
        return new JoystickButton(this, 6);
    }
    
    /**
     * Gets the BACK CL3 button (button 7) as a Trigger.
     *
     * @return A Trigger representing button 7
     */
    public Trigger backL3Button() {
        return new JoystickButton(this, 7);
    }
    
    /**
     * Gets the BACK CL4 button (button 8) as a Trigger.
     *
     * @return A Trigger representing button 8
     */
    public Trigger backL4Button() {
        return new JoystickButton(this, 8);
    }
    
    /**
     * Gets the AL1 button (button 9) as a Trigger.
     *
     * @return A Trigger representing button 9
     */
    public Trigger algaeL1Button() {
        return new JoystickButton(this, 9);
    }
    
    /**
     * Gets the AL2 button (button 10) as a Trigger.
     *
     * @return A Trigger representing button 10
     */
    public Trigger algaeL2Button() {
        return new JoystickButton(this, 10);
    }
    
    /**
     * Gets the FLIP button (button 11) as a Trigger.
     *
     * @return A Trigger representing button 11
     */
    public Trigger flipButton() {
        return new JoystickButton(this, 11);
    }
    
    /**
     * Gets a button by its number as a Trigger.
     *
     * @param buttonNumber The button number (1-12)
     * @return A Trigger representing the specified button
     * @throws IllegalArgumentException if the button number is out of range
     */
    public Trigger getButton(int buttonNumber) {
        if (buttonNumber < 1 || buttonNumber > 11) {
            throw new IllegalArgumentException("Button number must be between 1 and 11");
        }
        return new JoystickButton(this, buttonNumber);
    }
}