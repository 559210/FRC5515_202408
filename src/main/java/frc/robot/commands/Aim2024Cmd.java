package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateController;
import frc.robot.subsystems.Aim2025.Aim2024;
import frc.robot.subsystems.Swerve;

import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class Aim2024Cmd extends Command {
    private Swerve s_Swerve;
    private Aim2024 m_Aim;

    public Aim2024Cmd(Aim2024 aimSubSystem, Swerve swerve) {
        this.m_Aim = aimSubSystem;
        s_Swerve = swerve;
        addRequirements(m_Aim);
        schedule();
    }

    @Override
    public void initialize() {
        m_Aim.init();
    }

    @Override
    public void execute() {
        this.m_Aim.update();
        // ChassisSpeeds s = new ChassisSpeeds(-0.5, 0, 0);
        // s_Swerve.driveRobotRelative(s, null);

        // System.out.println("================> rob pos: " + s_Swerve.getPose().toString());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[[[[[[[[[[[[[[ END ]]]]]]]]]]]]]] :: " + interrupted);
        if (interrupted) {
            SmartDashboard.putString("Aim2024Cmd", "interrupted");
        }
        this.m_Aim.cancelAimMoveCmd();
        Pose2d pos = s_Swerve.getPose();
        System.out.println("end pos: " + pos.toString());
    }

    @Override
    public boolean isFinished() {
        switch (this.m_Aim.getAimMoveCmdState()) {
            case AIM_MOVE_CMD_STATE_FINISHED:
                System.out.println("isfinish 1");
                SmartDashboard.putString("Aim2024Cmd", "AIM_MOVE_CMD_STATE_FINISHED");
                return true;
            case AIM_MOVE_CMD_STATE_CANCELED:
                System.out.println("isfinish 2");
                SmartDashboard.putString("Aim2024Cmd", "AIM_MOVE_CMD_STATE_CANCELED");
                return true;
            case AIM_MOVE_CMD_STATE_IDLE:
                System.out.println("isfinish 3");
                return false;
            case AIM_MOVE_CMD_STATE_RUNNING:
            // System.out.println("isfinish 4");
                return false;
            case AIM_MOVE_CMD_STATE_UNKOWN:
                // System.out.println("isfinish 5");
                SmartDashboard.putString("Aim2024Cmd", "AIM_MOVE_CMD_STATE_UNKOWN");
                return false;
        }
        System.out.println("isfinish 6");
        return false;
    }
}
