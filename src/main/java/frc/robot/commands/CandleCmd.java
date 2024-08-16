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

public class CandleCmd extends Command {

    private Candle candle;

    private boolean isDone = false;
    // private Timer ellapsedTime_Trigger = new Timer();

    public CandleCmd(Candle candleSubsys) {
        candle = candleSubsys;
        addRequirements(candleSubsys);
        schedule();
    }

    @Override
    public void initialize() {
        candle.red();
    }

    int count = 0;
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
