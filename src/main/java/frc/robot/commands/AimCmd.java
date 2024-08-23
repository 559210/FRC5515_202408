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
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class AimCmd extends Command {
    private Swerve s_Swerve;
    private Aim m_Aim;
    private Candle candle;

    private boolean isDone = false;
    // private Timer ellapsedTime_Trigger = new Timer();

    public AimCmd(Aim aimSubSystem, Candle candleSubsys, Swerve swerve) {
        this.m_Aim = aimSubSystem;
        s_Swerve = swerve;
        candle = candleSubsys;
        addRequirements(m_Aim);
        // addRequirements(s_Swerve);
        schedule();
    }

    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("Aim working", true);
        StateController.getInstance().isAutoAimming = true;
        candle.blue();
        isDone = false;
    }

    int count = 0;
    @Override
    public void execute() {
        if (!m_Aim.isTargetValid()) {
            // isDone = true;
            return;
        }
        StateController.getInstance().isAutoAimming = true;
        StateController sc = StateController.getInstance();
        double epsilon = 1;
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
        // SmartDashboard.putString("auto_aim", "----------- START -----------");
        // SmartDashboard.putNumber("auto_aim_count", count);
        // SmartDashboard.putNumber("auto_aim_trans", absTy);
        // SmartDashboard.putNumber("auto_aim_rot", absTx);
        if (absTx >= epsilon || absTy >= epsilon) {
 
            if (absTx < epsilon) {
                rotationVal = 0;
            }
            if (absTy < epsilon) {
                translationVal = 0;
            }
            double strafeVal = 0;
            candle.yellow();
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal), 
                rotationVal, 
                false,
                true
            );
        }
        else {
            candle.red();
            isDone = true;
        }
    }

    int interruptedCount = 0;
    @Override
    public void end(boolean interrupted) {
        interruptedCount++;
        // SmartDashboard.putNumber("Aim interrupted", interruptedCount);
        // SmartDashboard.putBoolean("Aim working2", false);
        candle.red();
        StateController.getInstance().isAutoAimming = false;
    }

    @Override
    public boolean isFinished() {
        // SmartDashboard.putBoolean("auto_aim_isDone", isDone);
        return isDone;
    }
}
