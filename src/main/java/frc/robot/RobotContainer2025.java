package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer2025 implements RobotContainerInterface {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);
    // private final Joystick tester = new Joystick(2);

    // Drive Controls
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 5);
    private final JoystickButton robotCentric = new JoystickButton(driver, 6);
    private final Swerve2025 s_Swerve = new Swerve2025();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer2025() {
        s_Swerve.setDefaultCommand(
            new ParallelCommandGroup(
            new TeleopSwerve2025(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
            // ,new IntakeCmd(c_intake, ()->driver.getRawAxis(2), ()->driver.getRawAxis(3))
            // ,new IntakeAimCmd(s_Swerve, c_intakeAim, c_intake, candle, ()->driver.getRawAxis(2), ()->driver.getRawAxis(3))
            // ,new CandleCmd(candle)
            )
        );

        // Configure the button bindings
        // configureButtonBindings();


        // LimelightHelpers.setLEDMode_PipelineControl("limelight-one");
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return s_Swerve.followPathPlannerAuto("test01");
    }

    public void print() {
        Pose2d pos = s_Swerve.getPose();
        System.out.println(String.format("pos2d: (%f, %f)", pos.getX(), pos.getY()));
    }
}
