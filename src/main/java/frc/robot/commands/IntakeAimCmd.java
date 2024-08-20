package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.IntakeAim.IntakeAim;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeAimCmd extends Command {
    private Intake m_Intake;
    private IntakeAim m_IntakeAim;
    private Swerve s_Swerve;
    private DoubleSupplier leftTriggerButton;
    private DoubleSupplier rightTriggerButton;
    private Candle m_candle;
    // private Timer ellapsedTime_Trigger = new Timer();

    public IntakeAimCmd(Swerve swerve, IntakeAim m_IntakeAimSubSystem, Intake m_IntakeSubsystem, Candle candle,
            DoubleSupplier LTButton,
            DoubleSupplier RTButton) {
        this.m_Intake = m_IntakeSubsystem;
        m_IntakeAim = m_IntakeAimSubSystem;
        m_candle = candle;
        s_Swerve = swerve;
        this.leftTriggerButton = LTButton;
        this.rightTriggerButton = RTButton;
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_IntakeAimSubSystem);
        // addRequirements(s_Swerve);
        schedule();
    }

    @Override
    public void initialize() {
        // ellapsedTime_Trigger.start();
        SmartDashboard.putNumber("intake aim rotVal", 0);
        SmartDashboard.putNumber("intake aim translationVal", 0);
    }

    protected void updateAutoAim() {
        if (!m_IntakeAim.isTargetValid()) {
            StateController.getInstance().intakeAimStop = true;
            return;
        }
        StateController sc = StateController.getInstance();
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

        SmartDashboard.putNumber("intake aim rotVal", rotationVal);
        SmartDashboard.putNumber("intake aim translationVal", translationVal);
        double strafeVal = 0;
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal),
                rotationVal,
                false,
                true);
    }


    int greenCount = 0;
    @Override
    public void execute() {
        // SmartDashboard.putNumber("ellapsedTime_leftTrigger",ellapsedTime_Trigger.get());
        // if(ellapsedTime_Trigger.hasElapsed(0.5)){
        // ellapsedTime_Trigger.reset();
        if (leftTriggerButton.getAsDouble() > 0.15) {
            // m_Intake.update(IntakeState.Reverse);
            if (m_Intake.intakeState != IntakeState.NoteReady && m_Intake.intakeState != IntakeState.Intake_sensored) {
                m_Intake.update(IntakeState.Intake);

            }
        }
        if (rightTriggerButton.getAsDouble() > 0.15) {
            if (m_Intake.intakeState != IntakeState.NoteReady && m_Intake.intakeState != IntakeState.Intake_sensored) {
                m_Intake.update(IntakeState.Intake);
                if (StateController.getInstance().intakeAimStop == false) {
                    StateController.getInstance().isAutoIntakeAimming = true;
                    m_candle.blue();
                    updateAutoAim();
                }
                else {
                    
                    StateController.getInstance().isAutoIntakeAimming = false;
                }
            }

        } else {
            StateController.getInstance().intakeAimStop = false;
            m_IntakeAim.reset();
        }
        if (rightTriggerButton.getAsDouble() <= 0.15 && leftTriggerButton.getAsDouble() <= 0.15) {
            m_Intake.update(IntakeState.Stop);
        }
        // }else{

        // }
        if (m_Intake.isTriggerOk()) {
            greenCount++;
            SmartDashboard.putNumber("greencount", greenCount);
            m_candle.blinkGreenThenHold();
        }
        else {
            m_candle.red();
        }
        m_Intake.update();
    }

    @Override
    public void end(boolean interrupted) {
        StateController.getInstance().isAutoAimming = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
