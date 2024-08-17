package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.IntakeAim.IntakeAim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeForPathPlannerCmd extends Command {
    private Intake m_Intake;

    // private Timer ellapsedTime_Trigger = new Timer();

    private IntakeAim m_IntakeAim;
    private Swerve s_Swerve;
    private Candle m_candle;

    private boolean isDone = false;
    private boolean isStart = false;

    double startTime = 0;
    double m_maxTime = 0;

    public IntakeForPathPlannerCmd(Swerve swerve, IntakeAim m_IntakeAimSubSystem, Intake m_IntakeSubsystem, Candle candle, double maxTime) {
        this.m_Intake = m_IntakeSubsystem;
        m_IntakeAim = m_IntakeAimSubSystem;
        m_IntakeAim = m_IntakeAimSubSystem;
        m_candle = candle;
        s_Swerve = swerve;
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_IntakeAimSubSystem);
        schedule();
        m_maxTime = maxTime;
    }

    int initCount = 0;
    @Override
    public void initialize() {
        initCount++;
        SmartDashboard.putNumber("==initcount", initCount);
        m_IntakeAim.reset();
        // ellapsedTime_Trigger.start();
        startTime = Timer.getFPGATimestamp();
        isStart = true;
        isDone = false;
    }

    protected void updateAutoAim() {
        if (!m_IntakeAim.isTargetValid()) {
            s_Swerve.drive(
                    new Translation2d(0, 0),
                    0,
                    false,
                    true);
            StateController.getInstance().intakeAimStop = true;
            return;
        }
        
        double epsilon = 0.5;
        double translationVal = 0;
        // double translationVal = m_IntakeAim.limelight_range_proportional(); // the
        // value's range is unknown! by majun
        double rotationVal = m_IntakeAim.limelight_aim_proportional(); // the value's range is unknown! by majun
        // double absTrans = Math.abs(translationVal);
        double absTx = Math.abs(StateController.getInstance().intakeAimTx);
        double absRot = Math.abs(rotationVal);

        if (absRot > epsilon) {
            translationVal = 0;
        } else {
            rotationVal = 0;
            translationVal = 0.5;
        }
        SmartDashboard.putNumber("=== abs rot", absTx);
        SmartDashboard.putNumber("intake aim rotVal", rotationVal);
        SmartDashboard.putNumber("intake aim translationVal", translationVal);
        double strafeVal = 0;
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal),
                rotationVal,
                false,
                true);
    }
    @Override
    public void execute() {
        if (isStart) {
            SmartDashboard.putString("===intake state", StateController.getInstance().intakeAimStop?"true":"false");
            if (m_Intake.intakeState != IntakeState.NoteReady && m_Intake.intakeState != IntakeState.Intake_sensored)
            {
                m_Intake.update(IntakeState.Intake);
                if (StateController.getInstance().intakeAimStop == false) {
                    StateController.getInstance().isAutoIntakeAimming = true;
                    updateAutoAim();
                }
                else {
                    StateController.getInstance().isAutoIntakeAimming = false;
                }
            }    

            // if (m_Intake.isSensored) {
            SmartDashboard.putString("===state", m_Intake.intakeState.toString());
            SmartDashboard.putBoolean("===isTriggerReady", m_Intake.isTriggerOk());
            if (m_Intake.intakeState == IntakeState.Stop || m_Intake.isTriggerOk()) {
                m_Intake.update(IntakeState.Stop);
                StateController.getInstance().intakeAimStop = false;
                m_IntakeAim.reset();
                isStart = false;
                isDone = true;

                m_candle.white();
                s_Swerve.drive(
                        new Translation2d(0, 0),
                        0,
                        false,
                        true);
            }

            if (Timer.getFPGATimestamp() - startTime >= m_maxTime) {
                m_Intake.update(IntakeState.Stop);
                m_IntakeAim.reset();
                isStart = false;
                isDone = true;
                m_candle.yellow();
                                s_Swerve.drive(
                        new Translation2d(0, 0),
                        0,
                        false,
                        true);
            }
            m_Intake.update();
        }
    }

    @Override
    public void end(boolean interrupted) {
        StateController.getInstance().isAutoIntakeAimming = false;
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
