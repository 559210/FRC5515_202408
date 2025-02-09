package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants2025;
import frc.robot.ControlPadHelper;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;
import frc.robot.subsystems.Aim2025.Aim2025;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Elevator2025.Elevator2025.EV_STATE;
import frc.robot.subsystems.MoveTo.MoveTo2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.utils.MiscUtils;

public class UpperSystem2025Cmd extends Command {
    private TurningArm2025 m_turningArm;
    private Elevator2025 m_elevator;
    private boolean isInitOk = false;
    private JoystickButton armBtn;
    private JoystickButton zeroBtn;

    public UpperSystem2025Cmd(TurningArm2025 turningArm, Elevator2025 elev, JoystickButton arm, JoystickButton zero) {
        armBtn = arm;
        zeroBtn = zero;
        this.m_turningArm = turningArm;
        this.m_elevator = elev;
        addRequirements(m_turningArm);
        schedule();

        armBtn.onTrue(new InstantCommand(() -> {
            m_elevator.setState(EV_STATE.BASE);
        }));
        zeroBtn.onTrue(new InstantCommand(() -> {
            m_elevator.setState(EV_STATE.ZERO);
        }));
    }

    @Override
    public void initialize() {
        m_turningArm.init();
        m_elevator.init();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
