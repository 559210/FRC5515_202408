package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Aim.Aim;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;
import frc.robot.subsystems.Trigger.ShootTrigger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.IntakeAim.IntakeAim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);
    // private final Joystick tester = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 5);
    private final JoystickButton robotCentric = new JoystickButton(driver, 6);
    // private final JoystickButton button_intake = new getRawAxis(3);
    // private final JoystickButton button_intakereverse = new getRawAxis(2);
    private final JoystickButton button_amp = new JoystickButton(driver, 3);
    private final JoystickButton button_trigger = new JoystickButton(driver, 1);


    private final JoystickButton button_auto_aim = new JoystickButton(driver, 4);

    private final JoystickButton button_flywheelCoasting = new JoystickButton(driver2, 4);
    private final JoystickButton button_flywheelStop = new JoystickButton(driver2, 2);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShootTrigger c_trigger = new ShootTrigger();
    private final Intake c_intake = new Intake(c_trigger);
    private final Shooter c_shooter = new Shooter(c_trigger);
    private final Aim c_aim = new Aim();
    private final IntakeAim c_intakeAim = new IntakeAim();
    private final Candle candle = new Candle();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new ParallelCommandGroup(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
            // ,new IntakeCmd(c_intake, ()->driver.getRawAxis(2), ()->driver.getRawAxis(3))
            ,new IntakeAimCmd(s_Swerve, c_intakeAim, c_intake, candle, ()->driver.getRawAxis(2), ()->driver.getRawAxis(3))
            ,new CandleCmd(candle)
            )
        );

        // Configure the button bindings
        configureButtonBindings();
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
        button_flywheelCoasting.onTrue(
            new InstantCommand(()->{
                new ShooterCmd(c_shooter, candle, ShooterState.Coasting);
            })
        );
        button_flywheelStop.onTrue(
            new InstantCommand(()->{
                new ShooterCmd(c_shooter, candle, ShooterState.Stop);
            })
        );
        button_trigger.onTrue(new InstantCommand(()->{
                new ShooterCmd(c_shooter, candle, ShooterState.Shootout);
            }));
        button_amp.onTrue(new InstantCommand(()->{
                new ShooterCmd(c_shooter, candle, ShooterState.ShootAmp);
            })
        );

        // button_auto_aim.onTrue(new InstantCommand(()->{
        //     new AimCmd(c_aim, candle, s_Swerve);
        // }));

        button_auto_aim.whileTrue(
            new AimCmd(c_aim, candle, s_Swerve)
        );

        // new JoystickButton(tester, 1).whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // new JoystickButton(tester, 2).whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // new JoystickButton(tester, 3).whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // new JoystickButton(tester, 4).whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        
        // return s_Swerve.followPathCommand("Test");
        NamedCommands.registerCommand("init", new InstantCommand(()->{
            new ShooterCmd(c_shooter, candle, ShooterState.Coasting);
        }));
        NamedCommands.registerCommand("aim", 
            new AimCmd(c_aim, candle, s_Swerve)
        );
        for (int i = 1; i <= 100; ++i) {
            NamedCommands.registerCommand("intake" + String.valueOf(i), 
                new IntakeForPathPlannerCmd(s_Swerve, c_intakeAim, c_intake, candle,3)
            );
        }
        // NamedCommands.registerCommand("intake1", 
        //     new IntakeForPathPlannerCmd(s_Swerve, c_intakeAim, c_intake, candle,3)
        // );
        // NamedCommands.registerCommand("intake2", 
        //     new IntakeForPathPlannerCmd(s_Swerve, c_intakeAim, c_intake, candle,3)
        // );
        NamedCommands.registerCommand("shoot", new InstantCommand(()->{
            new ShooterCmd(c_shooter, candle, ShooterState.ShootAmp);
        }));
        return s_Swerve.followPathPlannerAuto("testauto2");
        // return new exampleAuto(s_Swerve);
    }
}
