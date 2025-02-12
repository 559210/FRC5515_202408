package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants2025;
import frc.robot.ControlPadHelper;
import frc.robot.ControlPadHelper.ControlPadInfo;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;
import frc.robot.subsystems.Aim2025.Aim2025;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Elevator2025.Elevator2025.EV_STATE;
import frc.robot.subsystems.MoveTo.MoveTo2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.utils.MiscUtils;


public class UpperSystem2025Cmd extends Command {
    private enum STATE {
        NONE,
        ZERO,
        READY_FOR_LOAD_CORAL,
        READY_FOR_LOAD_BALL,
        L1,
        L2,
        L3,
        L4,
        BALL1,
    }

    private enum RUNNING_STATE {
        NEW_SET,
        RUNNING,
        DONE,
    }
    private STATE lastState = STATE.NONE;
    private STATE curState = STATE.ZERO;
    private RUNNING_STATE curRunningState = RUNNING_STATE.DONE;


    private TurningArm2025 m_turningArm;
    private Elevator2025 m_elevator;
    // TODO: additional subsystems: sensor for checking if we are carrying Coral; outtake coral; intake coral, intake ball, outtake ball

    private Trigger test_armBtn; // test only
    private Trigger test_zeroBtn;    // test only

    private Trigger resetBtn;
    private Trigger switchCoralnBallBtn;
    private Trigger aimCoralBtn;

    private final boolean isDebugEnabled = true;

    private boolean isCarryingCoralFromDebug = false;
    private boolean isCarryingBallFromDebug = false;

    public UpperSystem2025Cmd(
            TurningArm2025 turningArm, Elevator2025 elev, 
            Trigger resetBtn, Trigger switchCnB, Trigger aimCoral, 
            Trigger test_arm, Trigger test_zero
        ) 
    {
        if (test_arm != null) {
            this.test_armBtn = test_arm;
            this.test_armBtn.onTrue(new InstantCommand(() -> {
                m_elevator.setState(EV_STATE.BASE);
            }));
        }
        
        if (test_zero != null) {
            this.test_zeroBtn = test_zero;
            this.test_zeroBtn.onTrue(new InstantCommand(() -> {
                m_elevator.setState(EV_STATE.ZERO);
            }));
        }
        
        if (resetBtn != null) {
            this.resetBtn = resetBtn;
            this.resetBtn.onTrue(new InstantCommand(() -> {
                System.out.println("reset elevator and turning arm's cancoder position to 0");
                m_elevator.resetCancodePosition();
                m_turningArm.resetCancodePosition();
            }));
        }

        if (switchCnB != null) {
            this.switchCoralnBallBtn = switchCnB;
            this.switchCoralnBallBtn.onTrue(new InstantCommand(() -> {
                if (curState == STATE.READY_FOR_LOAD_CORAL) {
                    setState(STATE.READY_FOR_LOAD_BALL);
                }
                else if (curState == STATE.READY_FOR_LOAD_BALL) {
                    setState(STATE.READY_FOR_LOAD_CORAL);
                }
            }));
        }

        if (aimCoral != null) {
            this.aimCoralBtn = aimCoral;
            this.aimCoralBtn.onTrue(new InstantCommand(() -> {
                ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
                if (info == null) {
                    return;
                }
                if (info.level == 0) {
                    setState(STATE.L1);
                }
                else if (info.level == 1) {
                    setState(STATE.L2);
                }
                else if (info.level == 2) {
                    setState(STATE.L3);
                }
                else if (info.level == 3) {
                    setState(STATE.L4);
                }
            }));
        }

        initDebug();

        this.m_turningArm = turningArm;
        addRequirements(m_turningArm);

        this.m_elevator = elev;
        
        addRequirements(m_elevator);
        schedule();

    }

    void initDebug() {
        if (isDebugEnabled) {
            ControlPadHelper.DebugCtrl.onZero.onTrue(new InstantCommand(() -> {
                setState(STATE.ZERO);
            }));
            ControlPadHelper.DebugCtrl.onReadyForloadCoral.onTrue(new InstantCommand(() -> {
                setState(STATE.READY_FOR_LOAD_CORAL);
            }));
            ControlPadHelper.DebugCtrl.onReadyForloadBall.onTrue(new InstantCommand(() -> {
                setState(STATE.READY_FOR_LOAD_BALL);
            }));
            ControlPadHelper.DebugCtrl.onL1.onTrue(new InstantCommand(() -> {
                setState(STATE.L1);
            }));
            ControlPadHelper.DebugCtrl.onL2.onTrue(new InstantCommand(() -> {
                setState(STATE.L2);
            }));
            ControlPadHelper.DebugCtrl.onL3.onTrue(new InstantCommand(() -> {
                setState(STATE.L3);
            }));
            ControlPadHelper.DebugCtrl.onL4.onTrue(new InstantCommand(() -> {
                setState(STATE.L4);
            }));
            ControlPadHelper.DebugCtrl.onBall1.onTrue(new InstantCommand(() -> {
                setState(STATE.BALL1);
            }));
            ControlPadHelper.DebugCtrl.onLoadCoral.onTrue(new InstantCommand(() -> {
                System.out.println("-------------> UpperSystem2025Cmd: onLoadCoral DEBUG: true");
                isCarryingCoralFromDebug = true;
            })).whileFalse(new InstantCommand(() -> {
                System.out.println("-------------> UpperSystem2025Cmd: onLoadCoral DEBUG false");
                isCarryingCoralFromDebug = false;
            }));
            ControlPadHelper.DebugCtrl.onLoadBall.onTrue(new InstantCommand(() -> {
                System.out.println("-------------> UpperSystem2025Cmd: onLoadBall DEBUG: true");
                isCarryingBallFromDebug = true;
            })).whileFalse(new InstantCommand(() -> {
                System.out.println("-------------> UpperSystem2025Cmd: onLoadBall DEBUG false");
                isCarryingBallFromDebug = false;
            }));
        }
    }

    @Override
    public void initialize() {
        // runs every time when roborio is enabled
        setState(STATE.READY_FOR_LOAD_CORAL);
    }

    @Override
    public void execute() {
        runState();
        updateState();
        telemetry();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("UpperSystem2025Cmd end");
        m_elevator.onDisable();
        m_turningArm.onDisable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // public void onDisable() {
    //     m_elevator.onDisable();
    //     m_turningArm.onDisable();
    // }

    private boolean getIsCarryingCarol() {
        // TODO: get this from sesnor
        if (isDebugEnabled) {
            return isCarryingCoralFromDebug;
        }
        return false;
    };

    private boolean getIsCarryingBall() {
        // TODO: get this from sesnor
        if (isDebugEnabled) {
            return isCarryingBallFromDebug;
        }
        return false;
    }

    private void setState(STATE newState) {
        System.out.println("UpperSystem2025Cmd::setState: try set: " + curState + " -> " + newState + " isCarryingCoral: " + getIsCarryingCarol() + " isCarryingBall: " + getIsCarryingBall()); 
        if (curState == newState) {
            return;
        }

        if (curState == STATE.ZERO && newState != STATE.READY_FOR_LOAD_CORAL) {
            // if we are in zero state, we can only go to READY_FOR_LOAD_CORAL state
            return;
        }

        if (newState == STATE.L1 || curState == STATE.L2 || curState == STATE.L3 || curState == STATE.L4) {
            if (curState != STATE.READY_FOR_LOAD_CORAL
                && curState != STATE.L1
                && curState != STATE.L2
                && curState != STATE.L3
                && curState != STATE.L4) 
            {
                // if we are not in READY_FOR_LOAD_CORAL, L1, L2, L3, L4, refuse to go to L1, L2, L3, L4
                return;
            }
            if (!getIsCarryingCarol() || getIsCarryingBall()) {
                // if we are not carrying Coral or are carrying ball, refuse to go to L1, L2, L3, L4
                return;
            }
        }
        
        if (newState == STATE.BALL1) {
            if (curState != STATE.READY_FOR_LOAD_BALL
            || curState != STATE.BALL1) 
            {
                // if we are not in READY_FOR_LOAD_BALL, refuse to go to BALL1
                return;
            }
            if (!getIsCarryingBall() || getIsCarryingCarol()) {
                // if we are not carrying ball or are carrying coral, refuse to go to BALL1
                return;
            }
        }

        if (newState == STATE.READY_FOR_LOAD_CORAL) {
            if (getIsCarryingBall() || getIsCarryingCarol()) {
                // if we are carrying ball or coral, refuse to go to READY_FOR_LOAD_CORAL
                return;
            }
        }

        if (newState == STATE.READY_FOR_LOAD_BALL) {
            if (getIsCarryingBall() || getIsCarryingCarol()) {
                // if we are carrying ball or coral, refuse to go to READY_FOR_LOAD_BALL
                return;
            }
        }

        curRunningState = RUNNING_STATE.NEW_SET;
        lastState = curState;
        curState = newState;

        System.out.println("UpperSystem2025Cmd::setState: try set: comfirmed");
    }

    private void runState() {
        switch (curState) {
            case ZERO:
                updateStateZero();
                break;
            case READY_FOR_LOAD_CORAL:
                updateStateReadyForLoadCoral();
                break;
            case L1:
                updateStateL1();
                break;
            case L2:
                updateStateL2();
                break;
            case L3:
                updateStateL3();
                break;
            case L4:
                updateStateL4();
                break;
            default:
                break;
        }
    }

    private void updateState() {
        // check if state should be changed
        // some state change bind to trigger. these triggers are set in the constructor
        if (lastState == STATE.ZERO && curRunningState != RUNNING_STATE.DONE) {
            // we are going from zero to other state, we should not change state. It's dangerous!!!!!!!
            return;
        }
        switch (curState) {
            case ZERO:
                break;
            case READY_FOR_LOAD_CORAL:
                if (getIsCarryingCarol()) {
                    setState(STATE.L1);
                }
                break;
            case READY_FOR_LOAD_BALL:
                if (getIsCarryingBall()) {
                    setState(STATE.BALL1);
                }
                break;
            case L1:
            case L2:
            case L3:
            case L4:
                if (!getIsCarryingCarol()) {
                    setState(STATE.READY_FOR_LOAD_CORAL);
                }
                break;
            case BALL1:
                if (!getIsCarryingBall()) {
                    setState(STATE.READY_FOR_LOAD_BALL);
                }
                break;
            default:
                break;
        }
    }
    private void updateStateZero() {
        switch (curRunningState) {
            case NEW_SET:
                // make elevator go to zero first.
                m_elevator.setState(EV_STATE.ZERO);
                curRunningState = RUNNING_STATE.RUNNING;
                break;
            case RUNNING:
                if (m_elevator.getState() != EV_STATE.ZERO) {
                    // something is wrong, we should not be here
                    System.out.println("UpperSystem2025Cmd::updateStateZero: elevator is not in zero state");
                    return;
                }
                if ( m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                    m_turningArm.setState(TA_STATE.ZERO);   // state can be set repeatedly

                    if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE) {
                        curRunningState = RUNNING_STATE.DONE;
                    }
                }
                break;
            case DONE:
                break;
        }
    }

    private void updateStateReadyForLoadCoral() {
        if (lastState == STATE.ZERO) {
            switch (curRunningState) {
                case NEW_SET:
                    m_turningArm.setState(TA_STATE.BASE);
                    curRunningState = RUNNING_STATE.RUNNING;
                    break;
                case RUNNING:
                    if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE) {
                        curRunningState = RUNNING_STATE.DONE;
                    }
                    break;
                case DONE:
                    break;
            }
        }
        else {
            switch (curRunningState) {
                case NEW_SET:
                    m_turningArm.setState(TA_STATE.BASE);
                    m_elevator.setState(EV_STATE.BASE);
                    curRunningState = RUNNING_STATE.RUNNING;
                    break;
                case RUNNING:
                    if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE &&
                        m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                        curRunningState = RUNNING_STATE.DONE;
                    }
                    break;
                case DONE:
                    break;
            }
        }
    }

    private void updateStateL1() {
        switch (curRunningState) {
            case NEW_SET:
                m_turningArm.setState(TA_STATE.L1);
                m_elevator.setState(EV_STATE.L1);
                curRunningState = RUNNING_STATE.RUNNING;
                break;
            case RUNNING:
                if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE &&
                    m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                    curRunningState = RUNNING_STATE.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

    private void updateStateL2() {
        switch (curRunningState) {
            case NEW_SET:
                m_turningArm.setState(TA_STATE.L2);
                m_elevator.setState(EV_STATE.L2);
                curRunningState = RUNNING_STATE.RUNNING;
                break;
            case RUNNING:
                if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE &&
                    m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                    curRunningState = RUNNING_STATE.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

    private void updateStateL3() {
        switch (curRunningState) {
            case NEW_SET:
                m_turningArm.setState(TA_STATE.L3);
                m_elevator.setState(EV_STATE.L3);
                curRunningState = RUNNING_STATE.RUNNING;
                break;
            case RUNNING:
                if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE &&
                    m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                    curRunningState = RUNNING_STATE.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

    private void updateStateL4() {
        switch (curRunningState) {
            case NEW_SET:
                m_turningArm.setState(TA_STATE.L3);
                m_elevator.setState(EV_STATE.L3);
                curRunningState = RUNNING_STATE.RUNNING;
                break;
            case RUNNING:
                if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE &&
                    m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                    curRunningState = RUNNING_STATE.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

    protected void telemetry() {
        SmartDashboard.putString("US2025Cmd_1", "state: " + curState + " running state: " + curRunningState);
        SmartDashboard.putString("US2025Cmd_2", "isCarryingCoral: " + getIsCarryingCarol() + " isCarryingBall: " + getIsCarryingBall());
    }
}
