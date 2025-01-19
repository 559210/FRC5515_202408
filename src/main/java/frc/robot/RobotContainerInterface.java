package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotContainerInterface {
    public Command getAutonomousCommand();
    public void telInit();
    public void autoInit();
    public void print();
}
