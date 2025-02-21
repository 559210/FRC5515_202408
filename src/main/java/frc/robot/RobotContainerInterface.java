package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotContainerInterface {
    public Command getAutonomousCommand();
    public void telInit();
    public void autoInit();
    public void testInit();
    public void onDisabled();
    public void update();           // only update when enabled
    public void updateAlways();     // update when enabled or disabled
}
