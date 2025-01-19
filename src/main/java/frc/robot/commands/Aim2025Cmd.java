package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;
import frc.robot.subsystems.Aim2025.Aim2025;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class Aim2025Cmd extends Command {
    private Swerve2025 s_Swerve;
    private Aim2025 m_Aim;

    public Aim2025Cmd(Aim2025 aimSubSystem, Swerve2025 swerve) {
        this.m_Aim = aimSubSystem;
        s_Swerve = swerve;
        addRequirements(m_Aim);
        schedule();
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        this.m_Aim.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putString("Aim2025Cmd", "interrupted");
        }
        this.m_Aim.cancelAimMoveCmd();
    }

    @Override
    public boolean isFinished() {
        switch (this.m_Aim.getAimMoveCmdState()) {
            case AIM_MOVE_CMD_STATE_FINISHED:
                SmartDashboard.putString("Aim2025Cmd", "AIM_MOVE_CMD_STATE_FINISHED");
                return true;
            case AIM_MOVE_CMD_STATE_CANCELED:
                SmartDashboard.putString("Aim2025Cmd", "AIM_MOVE_CMD_STATE_CANCELED");
                return true;
            case AIM_MOVE_CMD_STATE_IDLE:
                return false;
            case AIM_MOVE_CMD_STATE_RUNNING:
                return false;
            case AIM_MOVE_CMD_STATE_UNKOWN:
                SmartDashboard.putString("Aim2025Cmd", "AIM_MOVE_CMD_STATE_UNKOWN");
                return true;
        }
        return false;
    }
}
