package frc.robot.commands;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants2025;
import frc.robot.ControlPadHelper;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;
import frc.robot.subsystems.Aim2025.Aim2025;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.MoveTo.MoveTo2025;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.utils.MiscUtils;

public class WaitShooterCmd2025 extends Command {
    private Elevator2025 m_elevator;

    public WaitShooterCmd2025(Elevator2025 elevator)
    {
        m_elevator = elevator;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // System.out.println("====================> state: " + m_elevator.getCurRunningState().name());
        // if (m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
        if (m_elevator.getDifferFromTarget() <= 0.1) {
            return true;
        }
        return false;
    }
}
