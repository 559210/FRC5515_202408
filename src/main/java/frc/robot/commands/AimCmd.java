package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

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

    int count = 0;
    @Override
    public void execute() {
        StateController sc = StateController.getInstance();
        double epsilon = 0.5;
        double translationVal = m_Aim.limelight_range_proportional(); // the value's range is unknown! by majun
        double rotationVal = m_Aim.limelight_aim_proportional(); // the value's range is unknown! by majun
        double absTrans = Math.abs(translationVal);
        double absRot = Math.abs(rotationVal);

        double absTx = Math.abs(sc.aimTx);
        double absTy = Math.abs(sc.aimTy);

        // rotation first, then translate
        // if (absTx > epsilon) {
        //     // translationVal = 0;
        // }
        // else {
        //     rotationVal = 0;
        // }
        count++;
        SmartDashboard.putString("auto_aim", "----------- START -----------");
        SmartDashboard.putNumber("auto_aim_count", count);
        SmartDashboard.putNumber("auto_aim_trans", absTy);
        SmartDashboard.putNumber("auto_aim_rot", absTx);
        if (absTx >= epsilon || absTy >= epsilon) {
 
            if (absTx < epsilon) {
                rotationVal = 0;
            }
            if (absTy < epsilon) {
                translationVal = 0;
            }
            double strafeVal = 0;
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal), 
                rotationVal, 
                true,
                true
            );
        }
        else {

        }
            if (count >= 100) {
                isDone = true;
            }
            
    }

    @Override
    public void end(boolean interrupted) {
        StateController.getInstance().isAutoAimming = false;
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("auto_aim_isDone", isDone);
        return isDone;
    }
}
