package frc.robot.utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;


// create by majun 2025/2/13
// frc JoystickButton class is not working when disabled, so I create a new class JoystickButtonEx for that codes want to work when disabled by joystick.

/**
 * A {@link Trigger} that gets its state from a {@link GenericHID}.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class JoystickButtonEx extends Trigger {
    private static EventLoop eventLoop = new EventLoop();
    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick,
     *                     KinectStick, etc)
     * @param buttonNumber The button number (see
     *                     {@link GenericHID#getRawButton(int) }
     */
    public JoystickButtonEx(GenericHID joystick, int buttonNumber) {
        super(eventLoop, () -> joystick.getRawButton(buttonNumber));
        requireNonNullParam(joystick, "joystick", "JoystickButton");
    }

    // must be called in Robot::robotPeriodic() 
    public static void update() {
        eventLoop.poll();
    }
}
