package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Candle;
import frc.robot.commands.*;
import frc.robot.commands.SlightlyMoveCmd2025.DIR;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Aim2025.Aim2025;
import frc.robot.subsystems.Candle2025.Candle2025;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Intake2025.Intake2025;
import frc.robot.subsystems.MoveTo.MoveTo2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;


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
    private final Joystick tester = new Joystick(2);

    // Drive Controls
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton aimBtn = new JoystickButton(driver, 1);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 5);
    private final JoystickButton robotCentric = new JoystickButton(driver, 6);
    private final POVButton downButton = new POVButton(driver, 180);
    private final POVButton rightButton = new POVButton(driver, 90);
    private final POVButton upButton = new POVButton(driver, 0);
    private final POVButton leftButton = new POVButton(driver, 270);

    private final JoystickButton turningArmBtn = null; //new JoystickButton(driver,2);
    private final JoystickButton intakeBtn = new JoystickButton(driver,2);
    private final JoystickButton zeroStateBtn = null; //new JoystickButton(driver, 3);
    private final JoystickButton resetUpperCanCodePositionBtn = new JoystickButton(driver, 8);
    private final JoystickButton zeroUpperPosBtn = new JoystickButton(driver, 7);
    private final JoystickButton switchCoralnBallBtn = null; // new JoystickButton(driver, 5);
    private final Swerve2025 s_Swerve = new Swerve2025();

    // Aim2025 s_aim2025 = new Aim2025(s_Swerve);
    MoveTo2025 m_moveToSubSys = new MoveTo2025(s_Swerve);
    TurningArm2025 m_turningArm = new TurningArm2025();
    Elevator2025 m_elevator = new Elevator2025();
    Intake2025 m_intake = new Intake2025();
    Candle2025 m_candle = new Candle2025();

    private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer2025() {
        s_Swerve.setDefaultCommand(
            new ParallelCommandGroup(
                new TeleopSwerve2025(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    // ()->0, ()->0, ()->0,
                    () -> robotCentric.getAsBoolean()
                )
                , new UpperSystem2025Cmd(
                    m_turningArm, m_elevator, m_intake, m_candle, m_moveToSubSys,
                    resetUpperCanCodePositionBtn, zeroUpperPosBtn, switchCoralnBallBtn, aimBtn, intakeBtn,
                    turningArmBtn, zeroStateBtn
                )
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        GlobalConfig.init();
        ControlPadHelper.init();
        s_Swerve.configPathPlanner();
        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule();
        // LimelightHelpers.setLEDMode_PipelineControl("limelight-one");
        swerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    }

    public void telInit() {
        StateController.getInstance().useVisionOdometry = true;
        UpperSystem2025Cmd.inst.onRobotEnabled();
    } 
    public void autoInit() {
        StateController.getInstance().useVisionOdometry = true;
        UpperSystem2025Cmd.inst.onRobotEnabled();
    }

    public void onDisabled() {
        UpperSystem2025Cmd.inst.onRobotDisabled();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        rightButton.whileTrue(new SlightlyMoveCmd2025(s_Swerve, DIR.RIGHT));
        leftButton.whileTrue(new SlightlyMoveCmd2025(s_Swerve, DIR.LEFT));
        upButton.whileTrue(new SlightlyMoveCmd2025(s_Swerve, DIR.UP));
        downButton.whileTrue(new SlightlyMoveCmd2025(s_Swerve, DIR.DOWN));

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // aimBtn.whileTrue(new Aim2025Cmd(m_moveToSubSys, s_Swerve));

        // ControlPadHelper.goTargetTrigger.whileTrue(new Aim2025Cmd(m_moveToSubSys, s_Swerve));
        // ControlPadHelper.tapTrigger.whileTrue(new MoveTo2025Cmd(m_moveToSubSys, s_Swerve));

        ControlPadHelper.goTargetTrigger.onTrue(new InstantCommand(()-> {
            System.out.println("abc");
        }));

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
        return s_Swerve.followPathPlannerAuto("test01");
    }

    public void update() {
        Pose2d pos = s_Swerve.getPose();
        ControlPadHelper.publishRobotPos(pos);


        swerveStatePublisher.set(s_Swerve.getModuleStates());
    }

    public void updateAlways() {
        ControlPadHelper.update();
    }
}
