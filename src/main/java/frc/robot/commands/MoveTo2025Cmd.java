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
import frc.robot.subsystems.MoveTo.MoveTo2025;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.utils.MiscUtils;

public class MoveTo2025Cmd extends Command {
    private Swerve2025 s_Swerve;
    private MoveTo2025 m_moveToSubSys;
    private boolean isInitOk = false;

    public MoveTo2025Cmd(MoveTo2025 subSys, Swerve2025 swerve) {
        this.m_moveToSubSys = subSys;
        s_Swerve = swerve;
        addRequirements(m_moveToSubSys);
        schedule();
    }

    @Override
    public void initialize() {
        isInitOk = false;
        System.out.println("MoveTo2025Cmd init");
        ControlPadHelper.VirtualControl info = ControlPadHelper.getVirtualControl();
        if (info.isTap == false || (MiscUtils.compareDouble(info.x, 0) && MiscUtils.compareDouble(info.y, 0))) {
            System.out.println("MoveTo2025Cmd VirtualControl is not available");
            return;
        }
        Pose2d robotPos = s_Swerve.getPose();
        double degree = MiscUtils.calcAngle2Point(robotPos.getX(), robotPos.getY(), info.x, info.y);
        Pose2d targetPos = new Pose2d(info.x, info.y, Rotation2d.fromDegrees(degree));

        m_moveToSubSys.init(targetPos);

        isInitOk = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[[[[[[[[[[[[[[ END ]]]]]]]]]]]]]] :: " + interrupted);
        if (interrupted) {
            SmartDashboard.putString("MoveTo2025Cmd", "interrupted");
        }
        this.m_moveToSubSys.cancelAimMoveCmd();
        Pose2d pos = s_Swerve.getPose();
        System.out.println("end pos: " + pos.toString());
    }

    @Override
    public boolean isFinished() {
        if (isInitOk == false) {
            return true;
        }

        switch (this.m_moveToSubSys.getAimMoveCmdState()) {
            case MOVE_TO_CMD_STATE_FINISHED:
                System.out.println("isfinish 1");
                SmartDashboard.putString("MoveTo2025Cmd", "AIM_MOVE_CMD_STATE_FINISHED");
                return true;
            case MOVE_TO_CMD_STATE_RUNNING:
            // System.out.println("isfinish 4");
                return false;
            case MOVE_TO_CMD_STATE_UNKOWN:
                // System.out.println("isfinish 5");
                SmartDashboard.putString("MoveTo2025Cmd", "AIM_MOVE_CMD_STATE_UNKOWN");
                return false;
        }
        System.out.println("isfinish 6");
        return false;
    }
}
