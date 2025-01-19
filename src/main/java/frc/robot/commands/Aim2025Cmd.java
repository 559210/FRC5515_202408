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

    private boolean isDone = false;
    // private Timer ellapsedTime_Trigger = new Timer();

    public Aim2025Cmd(Aim2025 aimSubSystem, Swerve2025 swerve) {
        this.m_Aim = aimSubSystem;
        s_Swerve = swerve;
        addRequirements(m_Aim);
        // addRequirements(s_Swerve);
        schedule();
    }

    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("Aim working", true);

    }


    @Override
    public void execute() {
        this.m_Aim.getRobotRotateFromApriTag();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
