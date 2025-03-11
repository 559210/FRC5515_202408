package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants2025;
import frc.robot.ControlPadHelper;
import frc.robot.GlobalConfig;
import frc.robot.Robot;
import frc.robot.ControlPadHelper.ControlPadInfo;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;
import frc.robot.subsystems.Aim2025.Aim2025;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Candle2025.Candle2025;
import frc.robot.subsystems.Elevator2025.Elevator2025;
import frc.robot.subsystems.Elevator2025.Elevator2025.EV_STATE;
import frc.robot.subsystems.Intake2025.Intake2025;
import frc.robot.subsystems.MoveTo.MoveTo2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.ActionRunner;

public class UpperSystem2025Cmd extends Command {
    public static UpperSystem2025Cmd inst = null;

    private enum STATE {
        NONE,
        ZERO,
        READY_FOR_LOAD_CORAL,
        L1,
        L2,
        L3,
        L4,
        BALL1,      // equal to READY_FOR_LOAD_BALL
        BALL2,
    }

    // -1 down, 1 up, 0 unknow
    private int[][] table = new int[][]{
        //          NONE, ZERO,  READY_FOR_LOAD_CORAL, L1, L2, L3, L4, BALL1, BALL2
        new int[]{  0,    0,     0,                    0,  0,  0,  0,   0,     0     },// NONE
        new int[]{  0,    0,     -1,                   -1, -1, -1, -1, -1,    -1     },// ZERO
        new int[]{  0,    1,     0,                    -1, -1, -1, -1, -1,    -1     },// READY_FOR_LOAD_CORAL
        new int[]{  0,    1,     1,                    0,  -1, -1, -1, -1,    0     },// L1
        new int[]{  0,    1,     1,                    1,  0,  -1, -1, -1,    0     },// L2
        new int[]{  0,    1,     1,                    1,  1,  0,  -1, -1,    0     },// L3
        new int[]{  0,    1,     1,                    1,  1,  1,  0,  -1,    0     },// L4
        new int[]{  0,    0,     1,                    0,  0,  0,  0,   0,    1    },// BALL1
        new int[]{  0,    0,     1,                    0,  0,  0,  0,  -1 ,    0    },// BALL2
    };

    // private STATE[] STATE_UP_DIR = new STATE[] {
    //     STATE.ZERO,
    //     STATE.READY_FOR_LOAD_CORAL,
    //     STATE.L1,
    //     STATE.L2,
    //     STATE.L3,
    //     STATE.L4,
    // };
    

    // private STATE[] getStatePath(STATE from, STATE to) {
    //     int fromIndex = from.ordinal();
    //     int toIndex = to.ordinal();
    //     int dir = table[toIndex][fromIndex];
    //     if (dir == 0) {
    //         return new STATE[]{};
    //     }
    //     List<STATE> path = new ArrayList<STATE>();
    //     if (dir == 1) {
    //         for (int i = fromIndex + 1; i <= toIndex; i++) {
    //             path.add(STATE_UP_DIR[i]);
    //         }
    //     }
    //     else if (dir == -1) {
    //         for (int i = fromIndex - 1; i >= toIndex; i--) {
    //             path.add(STATE_UP_DIR[i]);
    //         }
    //     }
    //     return path.toArray(new STATE[0]);
    // }

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
    private Intake2025 m_intake;
    private Candle2025 m_candle;
    private MoveTo2025 m_moveTo;
    // TODO: additional subsystems: sensor for checking if we are carrying Coral;
    // outtake coral; intake coral, intake ball, outtake ball

    private Trigger test_armBtn; // test only
    private Trigger test_zeroBtn; // test only

    private Trigger resetCanCodePositionBtn;
    private Trigger resetToZeroPosBtn;
    // private Trigger switchCoralnBallBtn;
    private Trigger aimCoralBtn;
    private Trigger intakeTrigger;

    private Trigger elevatorTuningUpTrigger;
    private Trigger elevatorTuningDownTrigger;
    private Trigger armTuningUpTrigger;
    private Trigger armTuningDownTrigger;

    private Trigger catchBallTrigger;
    private Trigger toggleBallTrigger;

    private final boolean isDebugEnabled = true;

    private boolean isCarryingCoralFromDebug = false;
    private boolean isCarryingBallFromDebug = false;

    ActionRunner m_actionRunner = null;

    public UpperSystem2025Cmd(
            TurningArm2025 turningArm, Elevator2025 elev, Intake2025 intake, Candle2025 candle, MoveTo2025 moveto,
            Trigger resetToZeroPosBtn, Trigger switchCnB, Trigger aimCoral, Trigger intakeTrigger, Trigger _catchBall, Trigger _toggleBall,
            Trigger test_arm, Trigger test_zero) {

        inst = this;

        // for debug begin
        // switchCnB = null;
        test_arm = null;
        test_zero = null;
        // for debug end


        this.m_turningArm = turningArm;
        addRequirements(m_turningArm);

        this.m_elevator = elev;
        addRequirements(m_elevator);

        this.m_intake = intake;
        addRequirements(m_intake);

        this.m_candle = candle;
        addRequirements(m_candle);

        this.m_moveTo = moveto;
        addRequirements(m_moveTo);

        schedule();

        if (test_arm != null) {
            this.test_armBtn = test_arm;
            this.test_armBtn.onTrue(new InstantCommand(() -> {
                m_elevator.setState(EV_STATE.L4);
                /// m_turningArm.setState(TA_STATE.BASE);
            }));
        }

        if (test_zero != null) {
            this.test_zeroBtn = test_zero;
            this.test_zeroBtn.onTrue(new InstantCommand(() -> {
                m_elevator.setState(EV_STATE.ZERO);
                /// m_turningArm.setState(TA_STATE.ZERO);
            }));
        }

        if (resetToZeroPosBtn != null) {
            this.resetToZeroPosBtn = resetToZeroPosBtn;
            this.resetToZeroPosBtn.onTrue(new InstantCommand(() -> {
                setState(STATE.ZERO);
            }));
        }

        // if (switchCnB != null) {
        //     this.switchCoralnBallBtn = switchCnB;
        //     this.switchCoralnBallBtn.onTrue(new InstantCommand(() -> {
        //         if (curState == STATE.READY_FOR_LOAD_CORAL) {
        //             setState(STATE.READY_FOR_LOAD_BALL);
        //         } else if (curState == STATE.READY_FOR_LOAD_BALL) {
        //             setState(STATE.READY_FOR_LOAD_CORAL);
        //         }
        //     }));
        // }

        if (aimCoral != null) {
            this.aimCoralBtn = aimCoral;
            this.aimCoralBtn.onTrue(new InstantCommand(() -> {
                ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
                if (info == null) {
                    return;
                }

                Pose2d targetPos = Constants2025.aimPoses.get(info.aprilTagId);

                if (targetPos == null) {
                    System.out.println("Aim2025Cmd init targetPos is null");
                    return;
                }
                SmartDashboard.putNumber("Aim2025Cmd", info.aprilTagId);
                SmartDashboard.putString("Aim2025Cmd target", targetPos.toString());
        
                // StateController.getInstance().useVisionOdometry = false;
                m_moveTo.init(targetPos);
            })).whileFalse(new InstantCommand(() -> {
                this.m_moveTo.cancelAimMoveCmd();
            }));
        }

        if (intakeTrigger != null) {
            this.intakeTrigger = intakeTrigger;
            this.intakeTrigger.onTrue(new InstantCommand(() -> {
                if (curState == STATE.BALL1 || curState == STATE.BALL2) {
                    this.m_intake.toggleBallIntake();
                }
                else {
                    this.m_intake.toggleCoralIntake();
                }
                
            }));
        }

        if (_catchBall != null) {
            this.catchBallTrigger = _catchBall;
            this.catchBallTrigger.whileTrue(new InstantCommand(()-> {
                if ((curState == STATE.BALL1 || curState == STATE.BALL2) && curRunningState == RUNNING_STATE.DONE) {
                    m_elevator.setOffset(-1.5);
                    m_intake.toggleBallIntake(true);                    
                }

            })).onFalse(new InstantCommand(() -> {
                m_elevator.clearOffset();
                m_intake.toggleBallIntake(false);
            }));
        }

        if (_toggleBall != null) {
            this.toggleBallTrigger = _toggleBall;
            this.toggleBallTrigger.onTrue(new InstantCommand(() -> {
                setState(STATE.READY_FOR_LOAD_CORAL);
            }));
        }

        initDebug();
    }

    public void setElevatorTuningUpTrigger(Trigger t) {
        if (t == null || elevatorTuningUpTrigger != null) {
            return;
        }
        elevatorTuningUpTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            m_elevator.toggleTuningUp();
        }));
    }

    public void setElevatorTuningDownTrigger(Trigger t) {
        if (t == null || elevatorTuningDownTrigger != null) {
            return;
        }
        elevatorTuningDownTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            m_elevator.toggleTuningDown();
        }));
    }

    public void setArmTuningUpTrigger(Trigger t) {
        if (t == null || armTuningUpTrigger != null) {
            return;
        }
        armTuningUpTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            System.out.println("Arm up --------------------------------");
            m_turningArm.toggleTuningUp();
        }));
    }

    public void setArmTuningDownTrigger(Trigger t) {
        if (t == null || armTuningDownTrigger != null) {
            return;
        }
        armTuningDownTrigger = t;
        t.onTrue(new InstantCommand(() -> {
            m_turningArm.toggleTuningDown();
        }));
    }

    public void setResetCanCodePositionTrigger(Trigger t) {
        if (this.resetCanCodePositionBtn != null) {
            return;
        }
        if (t != null) {
            this.resetCanCodePositionBtn = t;
            this.resetCanCodePositionBtn.onTrue(new InstantCommand(() -> {
                if (Robot.inst.isTestEnabled()) {
                    System.out.println("reset elevator and turning arm's cancoder position to 0");
                    m_elevator.resetCancodePosition();
                    m_turningArm.resetCancodePosition();
                }
                else {
                    System.out.println("No No No, only working at TEST mode! reset elevator and turning arm's cancoder position to 0");
                }

            }));
        }
    }

    private Trigger lockElevatorTrigger;
    private Trigger unlockElevatorTrigger;
    public void setLockElevatorTrigger(Trigger t) {
        if (t == null || lockElevatorTrigger != null) {
            return;
        }
        lockElevatorTrigger = t;
        lockElevatorTrigger.onTrue(new InstantCommand(() -> {
            m_elevator.lockMotor();
            m_turningArm.unlockMotor();
        }));
    }

    public void setUnlockElevatorTrigger(Trigger t) {
        if (t == null || unlockElevatorTrigger != null) {
            return;
        }
        unlockElevatorTrigger = t;
        unlockElevatorTrigger.onTrue(new InstantCommand(() -> {
            m_elevator.unlockMotor();
            m_turningArm.unlockMotor();
        }));
    }

    void initDebug() {
        if (isDebugEnabled) {
            // ControlPadHelper.DebugCtrl.onZero.onTrue(new InstantCommand(() -> {
            //     setState(STATE.ZERO);
            // }));
            ControlPadHelper.DebugCtrl.onReadyForloadCoral.onTrue(new InstantCommand(() -> {
                setState(STATE.READY_FOR_LOAD_CORAL);
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
            ControlPadHelper.DebugCtrl.onBall2.onTrue(new InstantCommand(() -> {
                setState(STATE.BALL2);
            }));

            ControlPadHelper.DebugCtrl.shoot.onTrue(new InstantCommand(() -> {
                startShoot();
            }));
            // ControlPadHelper.DebugCtrl.onLoadCoral.onTrue(new InstantCommand(() -> {
            //     isCarryingCoralFromDebug = true;
            // })).whileFalse(new InstantCommand(() -> {
            //     isCarryingCoralFromDebug = false;
            // }));
            // ControlPadHelper.DebugCtrl.onLoadBall.onTrue(new InstantCommand(() -> {
            //     isCarryingBallFromDebug = true;
            // })).whileFalse(new InstantCommand(() -> {
            //     isCarryingBallFromDebug = false;
            // }));
        }
    }

    @Override
    public void initialize() {

        if (Robot.inst.isTeleop() || Robot.inst.isAutonomous())
        {
            System.out.println("UpperSystemCmd init in telop or auto");
            m_elevator.init();
            m_turningArm.init(); 
            m_intake.init();
            m_candle.init();
            setState(STATE.READY_FOR_LOAD_CORAL);
        }
        else if (Robot.inst.isTest()){
            System.out.println("UpperSystemCmd init in test");
            m_elevator.initInTestMode();
            m_turningArm.initInTestMode();
        }
    }

    @Override
    public void execute() {
        runState();
        updateState();
        updateLeds();
        telemetry();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("UpperSystemCmd END ======>" + interrupted);
        m_elevator.onDisable();
        m_turningArm.onDisable();
        m_candle.onDisable();
    }
    

    @Override
    public boolean isFinished() {
        return false;
    }

    // public void onDisable() {
    // m_elevator.onDisable();
    // m_turningArm.onDisable();
    // }

    private boolean getIsCarryingBall() {
        // TODO: get this from sesnor
        // if (isDebugEnabled) {
        //     return isCarryingBallFromDebug;
        // }
        return false;
    }

    private boolean getIsCarryingCoral() {
        if (m_intake.getIsCarryingCoral()) {
            return true;
        }

        // if (isDebugEnabled) {
        //     return isCarryingCoralFromDebug;
        // }

        return false;
    }

    private void setState(STATE newState) {
        System.out.println("UpperSystem2025Cmd::setState: try set: " + curState + " -> " + newState
                + " isCarryingCoral: " + getIsCarryingCoral() + " isCarryingBall: " + getIsCarryingBall());
        if (curState == newState) {
            return;
        }

        if (curState == STATE.ZERO && newState != STATE.READY_FOR_LOAD_CORAL) {
            // if we are in zero state, we can only go to READY_FOR_LOAD_CORAL state
            return;
        }

        if (newState == STATE.L1 || newState == STATE.L2 || newState == STATE.L3 || newState == STATE.L4) {
            if (curState != STATE.READY_FOR_LOAD_CORAL
                    && curState != STATE.L1
                    && curState != STATE.L2
                    && curState != STATE.L3
                    && curState != STATE.L4) {
                // if we are not in READY_FOR_LOAD_CORAL, L1, L2, L3, L4, refuse to go to L1,
                // L2, L3, L4
                return;
            }
            if (!getIsCarryingCoral() || getIsCarryingBall()) {
                // if we are not carrying Coral or are carrying ball, refuse to go to L1, L2,
                // L3, L4
                return;
            }
        }

        if (newState == STATE.BALL1 || newState == STATE.BALL2) {
            if (curState != STATE.READY_FOR_LOAD_CORAL && curState != STATE.BALL1 && curState != STATE.BALL2) {
                // if we are not in READY_FOR_LOAD_BALL, refuse to go to BALL1
                return;
            }
            if (getIsCarryingCoral()) {
                // if we are not carrying ball or are carrying coral, refuse to go to BALL1
                return;
            }
        }

        if (newState == STATE.READY_FOR_LOAD_CORAL) {
            if (getIsCarryingBall() || getIsCarryingCoral()) {
                // if we are carrying ball or coral, refuse to go to READY_FOR_LOAD_CORAL
                return;
            }
        }

        // if (newState == STATE.BALL1 && curState == STATE.READY_FOR_LOAD_CORAL) {
        //     if (getIsCarryingBall() || getIsCarryingCoral()) {
        //         // if we are carrying ball or coral, refuse to go to READY_FOR_LOAD_BALL
        //         return;
        //     }
        // }

        curRunningState = RUNNING_STATE.NEW_SET;
        lastState = curState;
        curState = newState;

        // System.out.println("UpperSystem2025Cmd::setState: try set: comfirmed");
    }


    private int curRunningDir = 0;
    private void doStateAction_____OLD(TA_STATE taState, EV_STATE evState) {
        if (curRunningState == RUNNING_STATE.NEW_SET)
        {
            STATE sx = lastState;
            STATE sy = curState;
            // sx = STATE.ZERO;
            // sy = STATE.READY_FOR_LOAD_BALL;
            int tx = sx.ordinal();
            int ty = sy.ordinal(); //

            curRunningDir = table[ty][tx];
            System.out.println("tx: " + tx + " ty: " + ty);
            System.out.println("curRunningDir is " + curRunningDir);
        }

        if (curRunningDir == -1) {
            // down
            switch (curRunningState) {
                case NEW_SET:
                    m_elevator.setState(evState);   
                    curRunningState = RUNNING_STATE.RUNNING;
                    break;
                case RUNNING:
                    if (m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                        m_turningArm.setState(taState);
                        if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE) {
                            curRunningState = RUNNING_STATE.DONE;
                        }
                    }
                    break;
                case DONE:
                    break;
            }
        }
        else if (curRunningDir == 0) {
            switch (curRunningState) {
                case NEW_SET:
                    m_turningArm.setState(taState);
                    m_elevator.setState(evState);
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
        else if (curRunningDir == 1) {
            // up
            switch (curRunningState) {
                case NEW_SET:
                    m_turningArm.setState(taState);
                      
                    curRunningState = RUNNING_STATE.RUNNING;
                    break;
                case RUNNING:
                    if (m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE) {
                        m_elevator.setState(evState); 
                        if (m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE) {
                            curRunningState = RUNNING_STATE.DONE;
                        }
                    }
                    break;
                case DONE:
                    break;
            }
        }
    }


    private void doStateAction(TA_STATE taState, EV_STATE evState) {
        boolean isNeedDodge = false;
        boolean isBall2Down = false;
        if (curRunningState == RUNNING_STATE.NEW_SET)
        {
            STATE sx = lastState;
            STATE sy = curState;
            int tx = sx.ordinal();
            int ty = sy.ordinal(); //

            curRunningDir = table[ty][tx];
            System.out.println("doStateAction NEW_SET: lastState: " + lastState.name() + ", curState: " + curState.name());
            System.out.println("doStateAction NEW_SET: tx: " + tx + " ty: " + ty);
            System.out.println("doStateAction NEW_SET: curRunningDir is " + curRunningDir);

            if ((lastState != STATE.BALL1 && lastState != STATE.BALL2) &&
                (curState == STATE.BALL1 || curState == STATE.BALL2)) {
                isNeedDodge = false;
            }
            else if ((lastState == STATE.BALL1 || lastState == STATE.BALL2) &&
                (curState == STATE.BALL1 || curState == STATE.BALL2)) {
                isNeedDodge = false;
            }
            else if ((lastState == STATE.BALL1 || lastState == STATE.BALL2) &&
                (curState == STATE.READY_FOR_LOAD_CORAL)) {
                isNeedDodge = false;
                isBall2Down = true;
            }
            else {
                double[] dodgePosList = m_elevator.getDodgePosOrderFromUp2Down();
                double elevatorStartPos = m_elevator.getCurPos();
                double elevatorEndPos = m_elevator.getStatePos(evState);
                for (int i = 0; i < dodgePosList.length; i++) {
                    if (MiscUtils.isBeween(dodgePosList[i], elevatorStartPos, elevatorEndPos)) {
                        isNeedDodge = true;
                        break;
                    }
                }
            }


            System.out.println("doStateAction NEW_SET: isNeedDodge: " + isNeedDodge);
            if (m_actionRunner != null) {
                m_actionRunner.cancel();
            }
            m_actionRunner = new ActionRunner();

            if (curRunningDir == -1) {
                // down
                if (isNeedDodge) {
                    m_actionRunner.addQueueAction(
                        () -> { // init
                        m_turningArm.setState(TA_STATE.DODGE);
                        },
                        () -> { // update
                        },
                        () -> {},   // onCancel
                        () -> { // endCondition
                            return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                        }
                    );
                }
                m_actionRunner.addQueueAction(
                    () -> { // init
                        m_elevator.setState(evState);
                    },
                    () -> { // update
                    },
                    () -> {},   // onCancel
                    () -> { // endCondition
                        return m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE;
                    }
                )
                .addConditionAction(
                    () -> { // init
                        m_turningArm.setState(taState);
                    },
                    () -> { // update
                    },
                    () -> {},   // onCancel
                    // startCondition
                    isNeedDodge ?
                    () -> { 
                        return m_elevator.isCurPosBelow(Constants2025.Elevator.downDodgePos);
                    } : (isBall2Down ? () -> {
                        return m_elevator.isCurPosBelow(m_elevator.getStatePos(EV_STATE.BALL1));
                    } :
                    () -> true),
                    () -> { // endCondition
                        return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                    }
                )
                // .addQueueAction(                    
                //     () -> { // init
                //         m_turningArm.setState(taState);
                //     },
                //     () -> { // update
                //     },
                //     () -> {},   // onCancel
                //     () -> { // endCondition
                //         return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                // })
                .start();
            }
            else if (curRunningDir == 1) {
                // up
                if (isNeedDodge) {
                    m_actionRunner.addQueueAction(
                        () -> { // init
                        m_turningArm.setState(TA_STATE.DODGE);
                        },
                        () -> { // update
                        },
                        () -> {},   // onCancel
                        () -> { // endCondition
                            return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                        }
                    );
                }
                m_actionRunner.addQueueAction(
                    () -> { // init
                        m_elevator.setState(evState);
                    },
                    () -> { // update
                    },
                    () -> {},   // onCancel
                    () -> { // endCondition
                        return m_elevator.getCurRunningState() == Elevator2025.RUNNING_STATE.DONE;
                    }
                )
                .addConditionAction(
                    () -> { // init
                        m_turningArm.setState(taState);
                    },
                    () -> { // update
                    },
                    () -> {},   // onCancel
                    // startCondition
                    isNeedDodge ? 
                    () -> { 
                        return m_elevator.isCurPosUpper(Constants2025.Elevator.upDodgePos);
                    } : 
                    () -> true,
                    () -> { // endCondition
                        return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                    }
                )
                // .addQueueAction(                    
                //     () -> { // init
                //         m_turningArm.setState(taState);
                //     },
                //     () -> { // update
                //     },
                //     () -> {},   // onCancel
                //     () -> { // endCondition
                //         return m_turningArm.getCurRunningState() == TurningArm2025.RUNNING_STATE.DONE;
                //     }
                // )
                .start();
            }

            curRunningState = RUNNING_STATE.RUNNING;
        }


        if (curRunningState == RUNNING_STATE.RUNNING) {
            m_actionRunner.update();
            if (m_actionRunner.getIsDone()) {
                curRunningState = RUNNING_STATE.DONE;
            }
        }
    }

    private void runState() {
        switch (curState) {
            case ZERO:
                doStateAction(TA_STATE.ZERO, EV_STATE.ZERO);
                // updateStateZero();
                break;
            case READY_FOR_LOAD_CORAL:
                doStateAction(TA_STATE.BASE, EV_STATE.BASE);
                // updateStateReadyForLoadCoral();
                break;
            case L1:
                doStateAction(TA_STATE.L1, EV_STATE.L1);
                // updateStateL1();
                break;
            case L2:
                doStateAction(TA_STATE.L2, EV_STATE.L2);
                // updateStateL2();
                break;
            case L3:
                doStateAction(TA_STATE.L3, EV_STATE.L3);
                // updateStateL3();
                break;
            case L4:
                doStateAction(TA_STATE.L4, EV_STATE.L4);
                // updateStateL4();
                break;
            case BALL1:
                doStateAction(TA_STATE.BALL1, EV_STATE.BALL1);
                break;
            case BALL2:
                doStateAction(TA_STATE.BALL2, EV_STATE.BALL2);
                break;
            default:
                break;
        }
    }

    private void updateState() {
        // check if state should be changed
        // some state change bind to trigger. these triggers are set in the constructor
        if (lastState == STATE.ZERO && curRunningState != RUNNING_STATE.DONE) {
            // we are going from zero to other state, we should not change state. It's
            // dangerous!!!!!!!
            return;
        }
        switch (curState) {
            case ZERO:
                break;
            case READY_FOR_LOAD_CORAL:
                if (getIsCarryingCoral()) {
                    setState(STATE.L1);
                }
                break;
            case L1:
            case L2:
            case L3:
            case L4:
                if (!getIsCarryingCoral()) {
                    setState(STATE.READY_FOR_LOAD_CORAL);
                }
                break;
            default:
                break;
        }
    }

    private void updateLeds() {
        if (getIsCarryingCoral()) {
            this.m_candle.showCarryingCoral();
        }
        else {
            this.m_candle.showIdle();
        }

        ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            this.m_candle.clearLn();
        }
        else {
            if (info.level == -1) {
                // show aglea
            }
            else if (info.level == 0) {
                this.m_candle.showL1();
            }
            else {
                if (info.branch != 0) {
                    boolean isLeft = (info.branch == -1);
                    switch ((int)info.level) {
                        case 1:
                            m_candle.showL2(isLeft);
                            break;
                        case 2:
                            m_candle.showL3(isLeft);
                            break;
                        case 3:
                            m_candle.showL4(isLeft);
                            break;
                    }                    
                }
            }
            
        }

    }

    public void setStateLn() {
        ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            return;
        }
        STATE newState = STATE.NONE;
        switch ((int)info.level) {
            case 0:
                newState = STATE.L1;
                break;
            case 1:
                newState = STATE.L2;
                break;
            case 2:
                newState = STATE.L3;
                break;
            case 3:
                newState = STATE.L4;
                break;
        }
        if (newState != STATE.NONE) {
            setState(newState);
        }
    }

    public void setStateL1() {
        setState(STATE.L1);
    }

    public void setStateL2() {
        setState(STATE.L2);
    }

    public void setStateL3() {
        setState(STATE.L3);
    }

    public void setStateL4() {
        setState(STATE.L4);
    }

    public void startIntake() {
        m_intake.startIntake();
    }

    public void startShoot() {
        m_intake.startShoot();
    }

    public void setStateBall1() {
        setState(STATE.BALL1);
    }

    public void setStateBall2() {
        setState(STATE.BALL2);
    }


    protected void telemetry() {

        // DON'T DELETE BELOW SmartDashboard push, cause ControlPad is using them!!!
        // DON'T DELETE BELOW SmartDashboard push, cause ControlPad is using them!!!
        // DON'T DELETE BELOW SmartDashboard push, cause ControlPad is using them!!!
        SmartDashboard.putString("US2025Cmd_State", "state: " + curState + " running state: " + curRunningState);
        // SmartDashboard.putString("US2025Cmd_CarryingState",
        //     "isCarryingCoral: " + m_intake.getIsCarryingCarol() + " isCarryingBall: " + getIsCarryingBall());
        SmartDashboard.putString("US2025Cmd_CarryingState",
            "isCarryingCoral: " + getIsCarryingCoral() + " isCarryingBall: " + getIsCarryingBall());

        SmartDashboard.putString("US2025Cmd_Sub_Running_State", "arm: " + m_turningArm.getCurRunningState() + " elevator: " + m_elevator.getCurRunningState() + " elevator state: " + m_elevator.getState());
                // DON'T DELETE UP CODES
        // DON'T DELETE UP CODES
        // DON'T DELETE UP CODES
    }
}
