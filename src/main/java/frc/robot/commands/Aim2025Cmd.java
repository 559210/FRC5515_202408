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

public class Aim2025Cmd extends Command {
    private Swerve2025 s_Swerve;
    private MoveTo2025 m_moveToSubSys;
    private boolean isInitOk = false;

    public Aim2025Cmd(MoveTo2025 subSys, Swerve2025 swerve) {
        this.m_moveToSubSys = subSys;
        s_Swerve = swerve;
        addRequirements(m_moveToSubSys);
        schedule();
    }

    @Override
    public void initialize() {
        isInitOk = false;
        System.out.println("Aim2025Cmd init");
        ControlPadHelper.ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            System.out.println("Aim2025Cmd init contorl info is null");
            return;
        }
        Pose2d targetPos = Constants2025.aimPoses.get(info.aprilTagId);

        if (targetPos == null) {
            System.out.println("Aim2025Cmd init targetPos is null");
            return;
        }
        SmartDashboard.putNumber("Aim2025Cmd", info.aprilTagId);
        SmartDashboard.putString("Aim2025Cmd target", targetPos.toString());

        // StateController.getInstance().useVisionOdometry = false;
        m_moveToSubSys.init(targetPos);

        isInitOk = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // StateController.getInstance().useVisionOdometry = true;
        System.out.println("[[[[[[[[[[[[[[ END ]]]]]]]]]]]]]] :: " + interrupted);
        if (interrupted) {
            SmartDashboard.putString("Aim2025Cmd", "interrupted");
        }
        this.m_moveToSubSys.cancelAimMoveCmd();
    }

    @Override
    public boolean isFinished() {
        if (isInitOk == false) {
            return true;
        }

        switch (this.m_moveToSubSys.getAimMoveCmdState()) {
            case MOVE_TO_CMD_STATE_FINISHED:
                System.out.println("isfinish 1");
                SmartDashboard.putString("Aim2025Cmd", "AIM_MOVE_CMD_STATE_FINISHED");
                return true;
            case MOVE_TO_CMD_STATE_RUNNING:
            // System.out.println("isfinish 4");
                return false;
            case MOVE_TO_CMD_STATE_UNKOWN:
                // System.out.println("isfinish 5");
                SmartDashboard.putString("Aim2025Cmd", "AIM_MOVE_CMD_STATE_UNKOWN");
                return false;
        }
        System.out.println("isfinish 6");
        return false;
    }
}
