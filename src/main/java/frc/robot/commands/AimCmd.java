package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Aim.Aim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class AimCmd extends Command {
    private Swerve s_Swerve;
    private Aim m_Aim;

    private boolean isDone = false;
    // private Timer ellapsedTime_Trigger = new Timer();

    public AimCmd(Aim aimSubSystem, Swerve swerve) {
        this.m_Aim = aimSubSystem;
        s_Swerve = swerve;
        addRequirements(m_Aim);
        addRequirements(s_Swerve);
        schedule();
    }

    @Override
    public void initialize() {
        StateController.getInstance().isAutoAimming = true;
    }

    @Override
    public void execute() {
        double epsilon = 0.01;
        double translationVal = m_Aim.limelight_range_proportional(); // the value's range is unknown! by majun
        double rotationVal = m_Aim.limelight_aim_proportional(); // the value's range is unknown! by majun
        double absTrans = Math.abs(translationVal);
        double absRot = Math.abs(rotationVal);

        // rotation first， then translate
        if (absRot > epsilon) {
            translationVal = 0;
        }
        else {
            rotationVal = 0;
        }
        if (absTrans >= epsilon || absRot >= epsilon) {
            double strafeVal = 0;
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal), 
                rotationVal, 
                true,
                true
            );
        }
        else {
            isDone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        StateController.getInstance().isAutoAimming = false;
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
