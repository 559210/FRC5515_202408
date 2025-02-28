
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

public class SlightlyMoveCmd2025 extends Command {
    public enum DIR {
        UP,
        RIGHT,
        DOWN,
        LEFT,
    }
    private Swerve2025 s_Swerve;
    private DIR m_dir;

    public SlightlyMoveCmd2025(Swerve2025 swerve, DIR dir) {
        s_Swerve = swerve;
        m_dir = dir;

        addRequirements(s_Swerve);
        schedule();
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("S_DIR_" + m_dir, 1);
    }

    @Override
    public void execute() {
        double data = 0.3;
        double x = 0;
        double y = 0;
        switch (m_dir) {
            case UP: x = data; break;
            case DOWN: x = -data; break;
            case LEFT: y = data; break;
            case RIGHT: y = -data; break;
        }
        s_Swerve.drive(new Translation2d(x, y), 0, false, true, 1);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("S_DIR_" + m_dir, 0);
    }

    @Override
    public boolean isFinished() {
    
        return false;
    }
}
